/**
 * @file mock_encoder_counter.hpp
 * @brief 单元测试专用：编码器计数值仿真器
 * @details
 * 将连续位置（单位：圈）量化为N位定时器无符号计数值，自动处理环形溢出回绕，
 * 模拟STM32正交编码定时器硬件计数行为，用于给 EncoderSpeedEstimator 提供仿真输入。
 *
 * 换算关系：
 *  count_floating = position * pulses_per_revolution
 *  对浮点计数值四舍五入后映射到 [0, wrap_max] 的环形计数空间
 *
 * 使用链路：
 *  MockFirstOrderPlant（输出连续位置） → MockEncoderCounter → uint32_t 计数值
 *  计数值送入 EncoderSpeedEstimator，完成测速闭环仿真验证
 *
 * @author Robot_FOC Project
 * @note
 *  - 仅用于PC端单元测试，**禁止编译进MCU产品固件**；
 *  - pulses_per_revolution 应当提前填入倍频后的总脉冲数（如原始线数×4）；
 *  - 不模拟编码器噪声、丢脉冲、毛刺，理想量化模型；如需噪声，在上层叠加。
 */
#ifndef ROBOT_FOC_TEST_MOCK_ENCODER_COUNTER_HPP
#define ROBOT_FOC_TEST_MOCK_ENCODER_COUNTER_HPP

#include <cstdint>

namespace robot_foc::test
{
    class MockEncoderCounter
    {
    private:
        // 倍频后每转总脉冲数
        std::uint32_t ppr_{0U};
        // 定时器计数寄存器位宽
        std::uint8_t  bits_{16U};
        // 计数器满量程最大值：（1 << bits_）-1
        std::uint32_t wrap_max_{0U};
        // 上一时刻浮点计数值，用于平滑连续位置推演
        float last_float_count_{0.0F};
        
    public:
        /**
         * @brief 构造编码器计数仿真器
         * @param pulses_per_revolution 一圈总脉冲（已包含倍频）
         * @param counter_bits 定时器计数寄存器位宽，如16U、32U
         */
        MockEncoderCounter(std::uint32_t pulses_per_revolution, std::uint8_t counter_bits) noexcept;
        
        // 禁止拷贝、移动
        MockEncoderCounter(const MockEncoderCounter&) = delete;
        MockEncoderCounter& operator=(const MockEncoderCounter&) = delete;
        MockEncoderCounter(MockEncoderCounter&&) = delete;
        MockEncoderCounter& operator=(MockEncoderCounter&&) = delete;
        
        /**
         * @brief 根据当前机械位置（圈）更新，输出硬件定时器计数值
         * @param position_rev 当前机械位置，单位：圈 可正可负
         * @return 环形无符号硬件计数值
         */
        [[nodiscard]] std::uint32_t update(float position_rev) noexcept;

        /**
         * @brief 重置仿真状态，清零内部计数基线
         */
        void reset() noexcept;
    };
} // namespace robot_foc::test

#endif
