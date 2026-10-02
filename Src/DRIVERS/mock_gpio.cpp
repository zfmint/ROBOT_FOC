#include "DRIVERS/mock_gpio.hpp"
#include "HAL/gpio.hpp"

namespace robot_foc::drivers
{
    MockGpio::MockGpio(robot_foc::hal::GpioPin pin)
        : Gpio(pin)
    { }

    MockGpio::~MockGpio() noexcept{
        if (is_initialized()){
            (void)deinit();
        }
    }

    bool MockGpio::init() noexcept{
        if (!mark_initialized())
        {
            return false;
        }
        level_ = robot_foc::hal::GpioLevel::Low;
        mode_ = robot_foc::hal::GpioMode::Input;
        return true;
    }

    bool MockGpio::deinit() noexcept{
        // 未初始化，重复deinit返回false
        if(!is_initialized())
        {
            return false;
        }
        return mark_deinitialized();    }
    
    bool MockGpio::setHigh() noexcept{
        if (is_initialized() && mode_ == robot_foc::hal::GpioMode::Output){
            level_ = robot_foc::hal::GpioLevel::High;
            return true;
        }
        return false;
    }

    bool MockGpio::setLow() noexcept{
        if (is_initialized() && mode_ == robot_foc::hal::GpioMode::Output){
            level_ = robot_foc::hal::GpioLevel::Low;
            return true;
        }
        return false;
    }

    bool MockGpio::setMode(robot_foc::hal::GpioMode mode) noexcept
    {
        if (!is_initialized())
        {
            return false;
        }
        mode_ = mode;
        return true;
    }

    bool MockGpio::readLevel(robot_foc::hal::GpioLevel& out_level) const noexcept{
        if (is_initialized())
        {
            out_level = level_;
            return true;
        }
        return false;
    }
} // namespace robot_foc::drivers
