/**
 * @file mock_gpio.cpp
 * @brief PC端MockGpio模拟GPIO实现，用于单元测试
 * @details
 *  用途：单元测试仿真，模拟外设生命周期与GPIO行为，不操作真实硬件寄存器；
 *  特性：
 *      1. 继承hal::Gpio，遵从Peripheral顶层生命周期规范，复用基类initialized_状态管理；
 *      2. 仿真引脚电平level_、引脚模式mode_；仅当已初始化且模式为Output时，set_high/set_low生效；
 *      3. 析构自动检测初始化状态，若未deinit则自动执行deinit，避免测试资源残留；
 *      4. 所有接口noexcept，与HAL硬件GPIO接口行为保持一致；
 *      5. init/deinit会重置模拟的电平、模式为默认状态。
 * @author Robot_FOC Project
 * @warning 仅用于PC单元测试仿真，**不可下载到嵌入式硬件控制真实GPIO**
 */

#include "DRIVERS/mock_gpio.hpp"

namespace robot_foc::drivers
{

    MockGpio::MockGpio(robot_foc::hal::GpioPin pin)
        : Gpio(pin)
    {
    }

    MockGpio::~MockGpio() noexcept
    {
        if (is_initialized())
        {
            (void)deinit();
        }
    }

    bool MockGpio::init() noexcept
    {
        if (!mark_initialized())
        {
            return false;
        }

        level_ = robot_foc::hal::GpioLevel::Low;
        mode_ = robot_foc::hal::GpioMode::Input;
        return true;
    }

    bool MockGpio::deinit() noexcept
    {
        if (!mark_deinitialized())
        {
            return false;
        }

        level_ = robot_foc::hal::GpioLevel::Low;
        mode_ = robot_foc::hal::GpioMode::Input;
        return true;
    }

    bool MockGpio::set_high() noexcept
    {
        if (!is_initialized() || mode_ != robot_foc::hal::GpioMode::Output)
        {
            return false;
        }

        level_ = robot_foc::hal::GpioLevel::High;
        return true;
    }

    bool MockGpio::set_low() noexcept
    {
        if (!is_initialized() || mode_ != robot_foc::hal::GpioMode::Output)
        {
            return false;
        }

        level_ = robot_foc::hal::GpioLevel::Low;
        return true;
    }

    bool MockGpio::set_mode(robot_foc::hal::GpioMode mode) noexcept
    {
        if (!is_initialized())
        {
            return false;
        }

        mode_ = mode;
        return true;
    }

    bool MockGpio::read_level(robot_foc::hal::GpioLevel& out_level) const noexcept
    {
        if (!is_initialized())
        {
            return false;
        }

        out_level = level_;
        return true;
    }

} // namespace robot_foc::drivers
