#include "HAL/peripheral.hpp"

namespace robot_foc::hal
{

    bool Peripheral::is_initialized() const noexcept
    {
        return initialized_;
    }

    bool Peripheral::mark_initialized() noexcept
    {
        if (initialized_)
        {
            return false;
        }

        initialized_ = true;
        return true;
    }

    bool Peripheral::mark_deinitialized() noexcept
    {
        if (!initialized_)
        {
            return false;
        }

        initialized_ = false;
        return true;
    }

} // namespace robot_foc::hal
