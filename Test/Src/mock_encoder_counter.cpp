/**
 * @file mock_encoder_counter.cpp
 * @brief 单元测试专用：编码器计数值仿真器实现
 * @details
 * 配套 mock_encoder_counter.hpp
 * 输入连续机械位置（圈），量化、环形取模，输出N位无符号计数值，模拟正交编码定时器。
 * 支持后续扩展：计数噪声、随机丢脉冲、A/B相故障、相位跳变等。
 *
 * @author Robot_FOC Project
 * @note
 *  - 仅PC单元测试使用，不参与MCU固件编译
 */

#include "mock_encoder_counter.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

namespace robot_foc::test
{
    MockEncoderCounter::MockEncoderCounter(std::uint32_t pulses_per_revolution, std::uint8_t counter_bits) noexcept
        : ppr_{pulses_per_revolution}
        , bits_{counter_bits}
    {
        if (bits_ < 8U || bits_ > 32U)
        {
            std::abort();
        }
        if (ppr_ == 0U)
        {
            std::abort();
        }
        if (bits_ == 32U)
        {
            wrap_max_ = std::numeric_limits<std::uint32_t>::max();
        }
        else
        {
            wrap_max_ = (1U << bits_) - 1U;
        }
        last_float_count_ = 0.0F;
    }

    std::uint32_t MockEncoderCounter::update(float position_rev) noexcept
    {
        // 根据位置算出理论浮点脉冲总数
        const float target_float_count = position_rev * static_cast<float>(ppr_);
        // 本次脉冲增量
        const float delta_float = target_float_count - last_float_count_;
        (void)delta_float;//暂不使用
        // 更新内部浮点计数值
        last_float_count_ = target_float_count;

        // 四舍五入取整
        const std::int64_t total_raw = std::llround(target_float_count);
        // 对环形量程取模，处理正负、多圈回绕
        const std::uint64_t range = static_cast<std::uint64_t>(wrap_max_) + 1U;
        
        // 将正负整数映射到[0, range - 1],规避C++负数取模符号问题
        std::int64_t val = total_raw % static_cast<std::int64_t>(range);
        if (val < 0)
        {
            val += static_cast<std::int64_t>(range);
        }
        return static_cast<std::uint32_t>(val);
    }

    void MockEncoderCounter::reset() noexcept
    {
        last_float_count_ = 0.0F;
    }
} // namespace robot_foc::test

