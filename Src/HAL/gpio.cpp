#include "HAL/gpio.hpp"

namespace robot_foc::hal
{
    Gpio::Gpio(GpioPin pin) noexcept
        : pin_(pin)
    { }
} // namespace robot_foc::hal
