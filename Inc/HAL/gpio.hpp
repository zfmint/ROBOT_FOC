/**
 * @file gpio.hpp
 * @brief GPIO外设抽象接口层定义
 *
 * 位于 HAL 抽象层，位于 Peripheral 顶层基类与具体硬件驱动之间：
 *
 * Peripheral（顶层外设基类：生命周期管理 init/deinit）
 *     └── Gpio（GPIO 抽象接口：只定义接口，与硬件无关）  ← 本文件
 *             └── MockGpio（PC 模拟实现）
 *             └── Stm32Gpio（未来单片机实现）
 *
 * 上层业务代码只依赖 Gpio 抽象接口，不关心底层具体实现，
 * 从而实现 PC 仿真与真实硬件之间的无缝替换。
 */
#ifndef GPIO_HPP
#define GPIO_HPP

#include "HAL/peripheral.hpp"
#include <cstdint>

namespace robot_foc::hal{
    //enum class 是强类型枚举
    enum class GpioMode : std::uint8_t{
        Input,
        Output
    };
    enum class GpioLevel : std::uint8_t{
        Low,
        High
    };
    /// @brief  GPIO引脚编号
    struct GpioPin
    {
        explicit constexpr GpioPin(uint8_t num) noexcept :num(num)
        { }
        std::uint8_t num;
    };
    class Gpio : public robot_foc::hal::Peripheral
    {
    private:
        /* data */
    public:
        explicit Gpio(GpioPin pin) noexcept;
        ~Gpio() override = default;
        
        //禁用拷贝、移动，外设对象禁止复制
        Gpio(const Gpio&) = delete;
        Gpio& operator=(const Gpio&) = delete;
        Gpio(Gpio&&) = delete;
        Gpio& operator=(Gpio&&) = delete;

        [[nodiscard]] virtual bool setHigh() noexcept = 0;
        [[nodiscard]] virtual bool setLow() noexcept = 0;
        [[nodiscard]] virtual bool setMode(GpioMode mode) noexcept = 0;
        [[nodiscard]] virtual bool readLevel(GpioLevel& out_level) const noexcept = 0;
        [[nodiscard]] GpioPin getPin() const noexcept{
            return pin_;
        }
        
    protected:
        GpioPin pin_;
    };
} // namespace robot_foc::hal

#endif
