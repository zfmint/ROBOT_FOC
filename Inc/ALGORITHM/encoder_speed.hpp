/**
 * @file encoder_speed.hpp
 * @brief 编码器转速估计器（模板编译期滤波窗口）
 * @details
 * 根据硬件定时器原始计数值，计算编码器转速RPM；支持N位计数器溢出回绕补偿。
 * 使用编译期固定窗口滑动平均滤波做转速平滑；无堆内存，适合MCU。
 *
 * 使用流程：
 *  1. 填充 EncoderSpeedConfig 配置结构体，调用 configure() 完成配置；is_configured()变为true
 *  2. 周期性调用 update()，传入当前硬件计数值与本次采样周期；
 *  3. 第一次update只建立计数基线，baseline_valid=false，不输出有效速度；
 *  4. 后续update输出 raw_rpm / filtered_rpm 至 EncoderSpeedSample。
 *
 * @tparam FilterWindow 滑动平均滤波窗口大小，必须 >= 2U
 *
 * @author Robot_FOC Project
 * @note
 *  - counter_bits：硬件定时器计数位宽，用于溢出补偿，合法范围8~32；
 *  - sample_period每次调用传入，支持可变采样周期，必须大于0秒；
 *  - 未配置状态调用update返回 ErrorNotReady；
 *  - 函数失败时，输出sample参数不会被修改；
 *  - 单元测试：Test/Src/test_encoder_speed.cpp
 *  - pulses_per_revolution：传入硬件倍频完成后的 CPR 总计数
 *  上层调用方自行完成 PPR× 硬件倍频计算；本底层类不做倍频换算
 * @warning 并发访问外部需要临界区保护，内部无同步
 */
#ifndef ROBOT_FOC_ALGORITHM_ENCODER_SPEED_HPP
#define ROBOT_FOC_ALGORITHM_ENCODER_SPEED_HPP

#include <cstddef>
#include <cstdint>

#include "ALGORITHM/sliding_average.hpp"
#include "COMMON/error_code.hpp"
#include "COMMON/units.hpp"

namespace robot_foc::algorithm
{
    /**
     * @brief 编码器转速估计器配置参数
     */
    struct EncoderSpeedConfig
    {
        // 编码器每转物理脉冲数
        std::uint32_t pulses_per_revolution{0U};
        // 硬件定时器计数位宽，用于溢出补偿，8-32
        std::uint8_t counter_bits{16U};
    };
    
    /**
     * @brief 单次update输出采样结果
     */
    struct EncoderSpeedSample
    {
        // 本次计数差分，带符号
        std::int32_t delta_counts{0};
        // 原始未滤波转速，RPM
        common::Rpm raw_rpm{0.0F};
        // 滑动后平均滤波后转速,RPM
        common::Rpm filtered_rpm{0.0F};
    };
    
    template<std::size_t FilterWindow>
    class EncoderSpeedEstimator
    {
        static_assert(FilterWindow >= 2U,"EncoderSpeedEstimator: FilterWindow must >=2");
    private:
        /**
         * @brief 计算带溢出补偿的计数值差分
         * @param current_count 当前硬件计数值
         * @param[out] delta_counts 输出带符号差分
         * @retval Ok 计算成功
         */
        [[nodiscard]] common::ErrorCode calculate_delta(
            std::uint32_t current_count,
            std::int32_t& delta_counts
        ) const noexcept;

        // 保存用户传入的配置参数
        EncoderSpeedConfig config_{};
        // 转速滑动平均滤波器，编译期窗口大小
        SlidingAverage<FilterWindow> speed_filter_{};

        // 上一次update的硬件计数值，计算差分
        std::uint32_t previous_count_{0U};
        // 每转总计数，有configure校验后缓存 pulses_per_revolution * 4
        std::uint32_t counts_per_revolution_{0U};

        // 判断configure是否配置成功
        bool configured_{false};
        // 判断已经保存有效的previous_count基线，可以计算差分
        bool baseline_valid_{false};

    public:
        EncoderSpeedEstimator() noexcept = default;
        ~EncoderSpeedEstimator() noexcept = default;

        //禁止移动、拷贝
        EncoderSpeedEstimator(const EncoderSpeedEstimator&) = delete;
        EncoderSpeedEstimator& operator=(const EncoderSpeedEstimator&) = delete;
        EncoderSpeedEstimator(EncoderSpeedEstimator&&) = delete;
        EncoderSpeedEstimator& operator=(EncoderSpeedEstimator&&) = delete;

        /**
         * @brief 配置转速估计器
         * @param config 配置结构体
         * @retval Ok 配置成功
         * @retval ErrorInvalidParam pulses_per_revolution为0/counter_bits不在8-32范围
         */
        [[nodiscard]] common::ErrorCode configure(const EncoderSpeedConfig& config) noexcept;

        /**
         * @brief 更新转速估计，送入最新硬件计数值
         * @param current_count 硬件定时器当前计数值
         * @param sample_period 本次采样周期，单位秒；必须>0
         * @param[out] sample 输出转速采样；函数失败时sample保持不变
         * @retval Ok 正常执行；首次调用仅建立基线，速度字段为0
         * @retval ErrorNotReady 尚未调用configure()配置
         * @retval ErrorInvalidParam sample_period <=0
         */
        [[nodiscard]] common::ErrorCode update(
            std::uint32_t current_count,
            common::Seconds sample_period,
            EncoderSpeedSample& sample
        ) noexcept;

        /**
         * @brief 重置估计器状态：基线、滤波窗口全部清空，配置保留不变
         */
        void reset() noexcept;

        /**
         * @brief 是否完成有效configure配置
         */
        [[nodiscard]] bool is_configured() const noexcept;

        /**
         * @brief 是否拥有有效上一次计数基线(已经至少完成一次差分计算)
         */
        [[nodiscard]] bool has_baseline() const noexcept;

        /**
         * @brief 获取每转总计数
         */
        [[nodiscard]] std::uint32_t counts_per_revolution() const noexcept;
    };
} // namespace robot_foc::algorithm




#endif
