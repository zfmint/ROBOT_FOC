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
 *      5. 生命周期管理：init/deinit控制初始化状态，重复deinit返回false；
 *  依赖：仅依赖HAL/gpio.hpp，无MCU寄存器、无外设驱动，可在PC编译运行单元测试
 * @author Robot_FOC Project
 * @warning 仅用于单元测试仿真，**不能在嵌入式目标板上用于控制真实硬件**
 */

#ifndef MOCK_GPIO_HPP
#define MOCK_GPIO_HPP

#include "HAL/gpio.hpp"

namespace robot_foc::drivers
{

    /**
     * @brief 不访问硬件寄存器的 GPIO 模拟实现。
     */
    class MockGpio final : public robot_foc::hal::Gpio
    {
    public:
        explicit MockGpio(robot_foc::hal::GpioPin pin);
        ~MockGpio() noexcept override;

        [[nodiscard]] bool init() noexcept override;
        [[nodiscard]] bool deinit() noexcept override;

        [[nodiscard]] bool set_high() noexcept override;
        [[nodiscard]] bool set_low() noexcept override;
        [[nodiscard]] bool set_mode(robot_foc::hal::GpioMode mode) noexcept override;
        [[nodiscard]] bool read_level(robot_foc::hal::GpioLevel& out_level) const noexcept override;

    private:
        robot_foc::hal::GpioLevel level_{robot_foc::hal::GpioLevel::Low};
        robot_foc::hal::GpioMode mode_{robot_foc::hal::GpioMode::Input};
    };

} // namespace robot_foc::drivers

#endif // MOCK_GPIO_HPP
