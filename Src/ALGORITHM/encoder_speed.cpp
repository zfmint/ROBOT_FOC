/**
 * @file encoder_speed.cpp
 * @brief 编码器转速估计器实现
 * @details
 * 配套 encoder_speed.hpp，实现计数值差分溢出补偿、转速计算、滑动平均滤波、状态复位。
 * 内部依赖：SlidingAverage 滑动平均滤波器
 *
 * 核心逻辑简述：
 *  1. configure：参数合法性校验，缓存 pulses_per_revolution
 *  2. calculate_delta：根据计数器位宽，自动处理向上/向下溢出，输出带符号计数值差分
 *  3. update：获取差分 → 计算原始RPM → 送入滑动平均滤波 → 填充输出sample
 *  4. reset：清空基线、滤波缓冲区，保留已有的configure配置
 *
 * @author Robot_FOC Project
 * @note
 *  - 本类无RTOS同步保护，多任务并发访问外部需要加临界区
 *  - sample_period 必须 > 0s，调用方保证采样周期稳定或可变均可
 *  - 滤波窗口在hpp内通过 kFilterWindow 固定，无需cpp修改
 */
#include <cmath>

#include "ALGORITHM/encoder_speed.hpp"

namespace robot_foc::algorithm
{
    /**
     * @brief 计算带溢出补偿的计数差分
     * @param current_count 当前计数值
     * @param[out] delta_counts 输出带符号差分结果
     * @retval Ok 计算完成
     */
    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::calculate_delta(
            std::uint32_t current_count,
            std::int32_t& delta_counts
        ) const noexcept
        {
            // 算出满量程最大值
            const std::uint32_t wrap_max = (1U << config_.counter_bits) - 1U;
            // 半量程，作为正常增量和溢出回绕的阈值
            const std::uint32_t half_range = wrap_max / 2U;
            // 原始差值，无符号减法天然支持回绕
            const std::uint32_t raw_diff = current_count - previous_count_;
            // 判断是不是“反转溢出”
            if (raw_diff > half_range)
            {
                // 反转回绕，真实差值为满量程减原始差值
                delta_counts = -static_cast<std::int32_t>(wrap_max + 1U - raw_diff);
            }
            else
            {
                delta_counts = static_cast<std::int32_t>(raw_diff);
            }
            return common::ErrorCode::Ok;
        }

    /**
     * @brief 配置编码器测速参数，校验并缓存配置
     * @param[in] config 输入配置结构体
     * @retval common::ErrorCode::Ok 配置成功
     * @retval common::ErrorCode::ErrorInvalidParam 参数非法
     */
    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::configure(const EncoderSpeedConfig& config) noexcept
    {
        // 参数校验
        if (config.pulses_per_revolution == 0U)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        if (config.counter_bits < 8U || config.counter_bits > 32U)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        config_ = config;
        counts_per_revolution_ = config.pulses_per_revolution;
        
        configured_ = true;

        return common::ErrorCode::Ok;
    }

    /*
    * @brief 更新转速估计，送入最新硬件计数值
    * @param current_count 硬件定时器当前计数值
    * @param sample_period 本次采样周期，单位秒；必须>0
    * @param[out] sample 输出转速采样；函数失败时sample保持不变
    * @retval Ok 正常执行；首次调用仅建立基线，速度字段为0
    * @retval ErrorNotReady 尚未调用configure()配置
    * @retval ErrorInvalidParam sample_period <=0
    */
    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::update(
        std::uint32_t current_count,
        common::Seconds sample_period,
        EncoderSpeedSample& sample
    ) noexcept
    {
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if (sample_period <= 0.0F)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        if (!baseline_valid_)
        {
            previous_count_ = current_count;
            baseline_valid_ = true;
            sample.delta_counts = 0;
            sample.raw_rpm = {0.0F};
            sample.filtered_rpm = {0.0F};
            return common::ErrorCode::Ok;
        }
        std::int32_t delta_counts = 0;
        const auto ret = calculate_delta(current_count,delta_counts);
        if (ret != common::ErrorCode::Ok)
        {
            return ret;
        }
        // 读取采样间隔
        const float dt = sample_period;
        // 转速公式： rpm = delta_counts / 每转脉冲数 / 采样周期 * 60
        const float raw_rpm_val = static_cast<float>(delta_counts)/static_cast<float>(counts_per_revolution_)/dt*60.0F;
        // 原始转速送入滑动平均滤波，得到滤波后转速
        float filtered_rpm_val = 0.0F;
        const auto filtered_ret = speed_filter_.calculate(raw_rpm_val,filtered_rpm_val);
        if (filtered_ret != true)
        {
            filtered_rpm_val = raw_rpm_val;
        }
        
        // 更新历史计数值，给下一次update做差分基准
        previous_count_ = current_count;
        // 填充输出结构体
        sample.delta_counts = delta_counts;
        sample.raw_rpm = common::Rpm{raw_rpm_val};
        sample.filtered_rpm = common::Rpm{filtered_rpm_val};

        return common::ErrorCode::Ok;
    }

    /**
     * @brief 重置估计器状态：基线、滤波窗口全部清空，配置保留不变
     */
    void EncoderSpeedEstimator::reset() noexcept
    {
        baseline_valid_ = false;
        previous_count_ = 0U;
        speed_filter_.reset();
    }

    /**
     * @brief 是否完成有效configure配置
     */
    [[nodiscard]] bool EncoderSpeedEstimator::is_configured() const noexcept
    {
        return configured_;
    }

    /**
     * @brief 是否拥有有效上一次计数基线(已经至少完成一次差分计算)
     */
    [[nodiscard]] bool EncoderSpeedEstimator::has_baseline() const noexcept
    {
        return baseline_valid_;
    }

    /**
     * @brief 获取每转总计数
     */
    [[nodiscard]] std::uint32_t EncoderSpeedEstimator::counts_per_revolution() const noexcept
    {
        return counts_per_revolution_;
    }

} // namespace robot_foc::algorithm
