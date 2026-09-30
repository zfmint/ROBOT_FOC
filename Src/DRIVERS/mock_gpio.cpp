#include "DRIVERS/mock_gpio.hpp"

namespace robot_foc::drivers
{
    MockGpio::MockGpio(uint8_t pinNumber)
        :pinNumber_(pinNumber)
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

    bool MockGpio::readLevel() const noexcept{
        if (is_initialized())
        {
            return level_;
        }
        
        return false;
    }
} // namespace robot_foc::drivers
