/**
 * @file mock_gpio.cpp
 * @brief MockGpio 模拟GPIO实现
 */
#include "DRIVERS/mock_gpio.hpp"

namespace robot_foc::drivers
{
using ErrorCode = typename robot_foc::hal::Gpio::ErrorCode;

MockGpio::MockGpio(hal::GpioPin pin) noexcept
    : Gpio(pin)
{}

[[nodiscard]] MockGpio::ErrorCode MockGpio::configure(const hal::GpioConfig& cfg) noexcept
{
    if (this->is_initialized())
    {
        return ErrorCode::ErrorAlreadyInit;
    }
    return this->set_config(cfg);
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::init() noexcept
{
    if (this->is_initialized())
    {
        return ErrorCode::ErrorAlreadyInit;
    }
    const auto ret_mark = this->mark_initialized();
    if (ret_mark != ErrorCode::Ok)
    {
        return ret_mark;
    }

    // 从基类读取配置，初始化仿真状态
    const auto& cfg = this->get_config();
    mode_ = cfg.mode;
    level_ = cfg.init_level;

    return ErrorCode::Ok;
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::deinit() noexcept
{
    if (!this->is_initialized())
    {
        return ErrorCode::ErrorNotReady;
    }
    const auto ret_mark = this->mark_deinitialized();
    if (ret_mark != ErrorCode::Ok)
    {
        return ret_mark;
    }

    // 仅重置运行时仿真状态；基类config_保留不变
    level_ = hal::GpioLevel::Low;
    mode_  = hal::GpioMode::Input;
    return ErrorCode::Ok;
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::set_high() noexcept
{
    if (!this->is_initialized())
    {
        return ErrorCode::ErrorNotReady;
    }
    if (mode_ != hal::GpioMode::OutputPushPull && mode_ != hal::GpioMode::OutputOpenDrain)
    {
        return ErrorCode::ErrorUnsupported;
    }
    level_ = hal::GpioLevel::High;
    return ErrorCode::Ok;
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::set_low() noexcept
{
    if (!this->is_initialized())
    {
        return ErrorCode::ErrorNotReady;
    }
    if (mode_ != hal::GpioMode::OutputPushPull && mode_ != hal::GpioMode::OutputOpenDrain)
    {
        return ErrorCode::ErrorUnsupported;
    }
    level_ = hal::GpioLevel::Low;
    return ErrorCode::Ok;
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::set_mode(hal::GpioMode mode) noexcept
{
    if (!this->is_initialized())
    {
        return ErrorCode::ErrorNotReady;
    }
    mode_ = mode;
    return ErrorCode::Ok;
}

[[nodiscard]] MockGpio::ErrorCode MockGpio::read_level(hal::GpioLevel& out_level) const noexcept
{
    if (!this->is_initialized())
    {
        return ErrorCode::ErrorNotReady;
    }
    out_level = level_;
    return ErrorCode::Ok;
}

void MockGpio::mock_force_input_level(hal::GpioLevel lv) noexcept
{
    // 只有输入模式才允许外部注入电平；输出引脚电平由set_high/set_low控制
    if (mode_ == hal::GpioMode::Input)
    {
        level_ = lv;
    }
}

} // namespace robot_foc::drivers