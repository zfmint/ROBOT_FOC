/**
 * @file gpio.cpp
 * @brief GPIO抽象基类实现文件
 * @details
 *  用途：实现Gpio基类构造函数；Gpio其余虚函数由派生类（硬件GPIO / MockGpio）实现
 *  特性：
 *      1. 继承自Peripheral外设基类，遵从统一外设生命周期；
 *      2. 构造函数接收GpioPin引脚对象，保存至protected成员pin_；
 *      3. 构造noexcept，无内存分配，适配嵌入式环境；
 *      4. 仅完成引脚绑定，**不执行硬件初始化**，init()需要单独调用。
 * @author Robot_FOC Project
 * @warning Gpio是抽象基类，不能直接实例化，仅用于派生
 */

#include "HAL/gpio.hpp"

namespace robot_foc::hal
{

    Gpio::Gpio(GpioPin pin) noexcept
        : pin_(pin)
    {
    }

} // namespace robot_foc::hal
