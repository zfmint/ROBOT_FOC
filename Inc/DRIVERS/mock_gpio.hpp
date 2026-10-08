/**
 * @file mock_gpio.hpp
 * @brief PC 端 GPIO 模拟驱动，用于单元测试
 * @details
 *  用途：PC平台单元测试，模拟MCU GPIO外设行为，不操作真实硬件寄存器；
 *  特性：
 *      1. 继承自hal层Gpio基类，接口与真实硬件GPIO完全对齐；
 *      2. 内部仅用内存变量保存引脚电平、模式状态，无硬件访问；
 *      3. 全部接口 noexcept，和底层HAL接口保持一致；
 *      4. final类，禁止再次继承，防止测试场景意外派生；
 *      5. 完整Peripheral生命周期管理：configure / init / deinit；
 *  依赖：仅依赖HAL/gpio.hpp，无MCU寄存器、无外设驱动，可在PC编译运行单元测试
 * @author Robot_FOC Project
 * @warning 仅用于单元测试仿真，**不能在嵌入式目标板上用于控制真实硬件**
 */

#ifndef ROBOT_FOC_DRIVERS_MOCK_GPIO_HPP
#define ROBOT_FOC_DRIVERS_MOCK_GPIO_HPP

#include <type_traits>

#include "HAL/gpio.hpp"

namespace robot_foc::drivers
{
    /**
     * @brief 不访问硬件寄存器的 GPIO 模拟实现。
     */
    class MockGpio final : public robot_foc::hal::Gpio
    {
    private:
        using ErrorCode = typename robot_foc::common::ErrorCode;

        robot_foc::hal::GpioLevel level_{robot_foc::hal::GpioLevel::Low};
        robot_foc::hal::GpioMode  mode_{robot_foc::hal::GpioMode::Input};
    public:
        explicit MockGpio(robot_foc::hal::GpioPin pin) noexcept;
        ~MockGpio() noexcept override = default;

        MockGpio(const MockGpio&) = delete;
        MockGpio& operator=(const MockGpio&) = delete;
        MockGpio(MockGpio&&) noexcept = delete;
        MockGpio& operator=(MockGpio&&) noexcept = delete;

        [[nodiscard]] ErrorCode configure(const robot_foc::hal::GpioConfig& cfg) noexcept override;

        [[nodiscard]] ErrorCode init() noexcept override;
        [[nodiscard]] ErrorCode deinit() noexcept override;

        [[nodiscard]] ErrorCode set_high() noexcept override;
        [[nodiscard]] ErrorCode set_low() noexcept override;
        [[nodiscard]] ErrorCode set_mode(robot_foc::hal::GpioMode mode) noexcept override;
        [[nodiscard]] ErrorCode read_level(robot_foc::hal::GpioLevel& out_level) const noexcept override;

        /// @brief 测试辅助接口：模拟外部硬件改变输入引脚电平，仅单元测试调用
        void mock_force_input_level(robot_foc::hal::GpioLevel lv) noexcept;
    };

    // 编译期校验 MockGpio：禁止拷贝、禁止移动
    static_assert(!std::is_copy_constructible_v<robot_foc::drivers::MockGpio>);
    static_assert(!std::is_copy_assignable_v<robot_foc::drivers::MockGpio>);
    static_assert(!std::is_move_constructible_v<robot_foc::drivers::MockGpio>);
    static_assert(!std::is_move_assignable_v<robot_foc::drivers::MockGpio>);

} // namespace robot_foc::drivers

#endif // ROBOT_FOC_DRIVERS_MOCK_GPIO_HPP
