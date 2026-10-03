/**
 * @file gpio.hpp
 * @brief GPIO 外设抽象接口。
 * @details
 *  用途：HAL层GPIO统一抽象，隔离上层业务与底层硬件寄存器；
 *  特性：
 *      1. 纯虚基类，定义统一GPIO操作接口，支持真实硬件GPIO与Mock模拟GPIO两套实现；
 *      2. 内置GpioMode、GpioLevel强类型枚举，底层固定为uint8_t，适配嵌入式寄存器；
 *      3. GpioPin强类型封装引脚号，避免裸uint8_t混淆引脚与其他数值；
 *      4. 禁止拷贝、禁止移动，外设对象不做值传递，仅传递引用；
 *      5. 全部硬件相关虚接口 noexcept，适配嵌入式环境；
 *  依赖：cstdint、HAL/peripheral.hpp，为所有GPIO外设的公共抽象层
 * @author Robot_FOC Project
 * @warning 本类为抽象基类，不可直接实例化；必须由硬件实现类或Mock模拟类继承实现
 */
#ifndef ROBOT_FOC_HAL_GPIO_HPP
#define ROBOT_FOC_HAL_GPIO_HPP

#include <cstdint>

#include "HAL/peripheral.hpp"

namespace robot_foc::hal
{

    enum class GpioMode : std::uint8_t
    {
        Input,
        Output
    };

    enum class GpioLevel : std::uint8_t
    {
        Low,
        High
    };

    /** @brief GPIO 引脚编号值类型。 */
    struct GpioPin
    {
        explicit constexpr GpioPin(std::uint8_t number) noexcept
            : number(number)
        {
        }

        std::uint8_t number;
    };

    /**
     * @brief GPIO 抽象接口。
     */
    class Gpio : public Peripheral
    {
    private:
        ///@brief 引脚编号
        GpioPin pin_;
        
    public:
        explicit Gpio(GpioPin pin) noexcept;
        ~Gpio() noexcept override = default;

        /// @brief 禁止拷贝、移动
        Gpio(const Gpio&) = delete;
        Gpio& operator=(const Gpio&) = delete;
        Gpio(Gpio&&) noexcept = delete;
        Gpio& operator=(Gpio&&) noexcept = delete;

        [[nodiscard]] virtual bool set_high() noexcept = 0;
        [[nodiscard]] virtual bool set_low() noexcept = 0;
        [[nodiscard]] virtual bool set_mode(GpioMode mode) noexcept = 0;
        [[nodiscard]] virtual bool read_level(GpioLevel& out_level) const noexcept = 0;

        [[nodiscard]] GpioPin get_pin() const noexcept
        {
            return pin_;
        }
    };

} // namespace robot_foc::hal

#endif // GPIO_HPP
