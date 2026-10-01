#include "DRIVERS/mock_gpio.hpp"
#include "HAL/gpio.hpp"

namespace robot_foc::drivers
{
    MockGpio::MockGpio(robot_foc::hal::GpioPin pin)
        :Gpio (pin)
    { }

    MockGpio::~MockGpio() noexcept{
        if (is_initialized()){
            (void)deinit();
        }
    }

    bool MockGpio::init() noexcept{
        if (is_initialized())
        {
            return true;
        }
        if (!mark_initialized())
        {
            return false;
        }
        level_ = false;
        return true;
    }

    bool MockGpio::deinit() noexcept{
        return mark_deinitialized();
    }
    
    bool MockGpio::setHigh() noexcept{
        if (is_initialized()){
            level_ = true;
            return true;
        }else{
            return false;
        }
    }

    bool MockGpio::setLow() noexcept{
        if (is_initialized()){
            level_ = false;
            return true;
        }else{
            return false;
        }
    }

    bool MockGpio::readLevel(bool& out_level) const noexcept{
        if (is_initialized())
        {
            out_level = level_;
            return true;
        }
        return false;
    }
} // namespace robot_foc::drivers
