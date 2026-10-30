/**
 * @file encoder_speed.cpp
 * @brief 编码器转速估计器实现
 * @details
 * 根据定时器计数差分计算转速，支持16位和32位计数器回绕、正反转判断、
 * 运行时正交倍频配置以及固定窗口滑动平均滤波。
 */

#include "ALGORITHM/encoder_speed.hpp"

#include <cmath>
#include <limits>

#include "error_code.hpp"

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::calculate_delta(
        std::uint32_t current_count,
        std::int32_t& delta_counts) const noexcept
    {
        // 计数器环总长度
        const std::uint64_t counter_modulus = 1ULL << config_.counter_bits;
        // 计数器最大值
        const std::uint64_t counter_max = counter_modulus - 1ULL;

        if ((static_cast<std::uint64_t>(current_count) > counter_max) ||
            (static_cast<std::uint64_t>(previous_count_) > counter_max))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        // 算出原始差值
        std::int64_t difference =
            static_cast<std::int64_t>(current_count) -
            static_cast<std::int64_t>(previous_count_);
        // 环形计数器总刻度数量
        const std::int64_t signed_modulus = static_cast<std::int64_t>(counter_modulus);
        // 半量程
        const std::int64_t half_range = signed_modulus / 2;

        // 相差正好半量程时无法唯一判断旋转方向。
        if ((difference == half_range) || (difference == -half_range))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        if (difference > half_range)
        {
            difference -= signed_modulus;
        }
        else if (difference < -half_range)
        {
            difference += signed_modulus;
        }

        if ((difference < static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min())) ||
            (difference > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        delta_counts = static_cast<std::int32_t>(difference);
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::configure(
        const EncoderSpeedConfig& config) noexcept
    {
        if (config.pulses_per_revolution == 0U)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        if (config.counter_bits < 8U || config.counter_bits > 32U)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        if ((config.quadrature_gain != 1U) &&
            (config.quadrature_gain != 2U) &&
            (config.quadrature_gain != 4U))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        if (config.pulses_per_revolution >
            (std::numeric_limits<std::uint32_t>::max() / config.quadrature_gain))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        config_ = config;
        counts_per_revolution_ = config.pulses_per_revolution * config.quadrature_gain;
        configured_ = true;
        reset();

        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode EncoderSpeedEstimator::update(
        std::uint32_t current_count,
        common::Seconds sample_period,
        EncoderSpeedSample& sample) noexcept
    {
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }

        const float period = sample_period;
        if (!std::isfinite(period) || (period <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        const std::uint64_t counter_max = (1ULL << config_.counter_bits) - 1ULL;
        if (static_cast<std::uint64_t>(current_count) > counter_max)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        if (!baseline_valid_)
        {
            previous_count_ = current_count;
            baseline_valid_ = true;
            sample = EncoderSpeedSample{};
            return common::ErrorCode::Ok;
        }

        std::int32_t delta_counts{0};
        const common::ErrorCode delta_result = calculate_delta(current_count, delta_counts);
        if (delta_result != common::ErrorCode::Ok)
        {
            return delta_result;
        }

        const float raw_rpm =
            (static_cast<float>(delta_counts) * 60.0F) /
            (static_cast<float>(counts_per_revolution_) * period);
        if (!std::isfinite(raw_rpm))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        float filtered_rpm{0.0F};
        if (!speed_filter_.calculate(raw_rpm, filtered_rpm))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        previous_count_ = current_count;
        // 填充输出结构体
        sample.delta_counts = delta_counts;
        sample.raw_rpm = raw_rpm;
        sample.filtered_rpm = filtered_rpm;

        return common::ErrorCode::Ok;
    }

    void EncoderSpeedEstimator::reset() noexcept
    {
        baseline_valid_ = false;
        previous_count_ = 0U;
        speed_filter_.reset();
    }

    [[nodiscard]] bool EncoderSpeedEstimator::is_configured() const noexcept
    {
        return configured_;
    }

    [[nodiscard]] bool EncoderSpeedEstimator::has_baseline() const noexcept
    {
        return baseline_valid_;
    }

    [[nodiscard]] std::uint32_t EncoderSpeedEstimator::counts_per_revolution() const noexcept
    {
        return counts_per_revolution_;
    }
} // namespace robot_foc::algorithm
