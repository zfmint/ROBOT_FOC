/**
 * @file mock_gpio.hpp
 * @brief PC端模拟GPIO外设实现，用于单元测试
 *
 * 继承关系：
 * Peripheral（顶层生命周期基类）
 *     └── Gpio（GPIO抽象接口）
 *             └── MockGpio（本类，PC仿真实现）
 *
 * 职责：
 * 1. 在PC平台模拟GPIO外设行为，**不访问任何硬件寄存器**，用于单元测试
 * 2. 实现Gpio抽象基类定义的全部接口，和未来单片机端Stm32Gpio接口完全对齐
 * 3. RAII生命周期：对象析构时，如果处于已初始化状态，自动执行deinit释放资源
 * 4. 内置电平状态level_，模拟引脚高低电平；非法操作保护：未初始化时操作直接返回失败
 *
 * 使用场景：
 * PC单元测试，在无硬件环境下验证上层业务逻辑，提前发现逻辑bug。
 */
#ifndef MOCK_GPIO_HPP
#define MOCK_GPIO_HPP

#include "HAL/peripheral.hpp"
#include "HAL/gpio.hpp"
#include <cstdint>

namespace robot_foc::drivers
{
    /**
     * @brief PC端模拟GPIO外设，用于单元测试
     * 继承Gpio基类，不操作真实硬件寄存器
     * @note RAII约束：对象销毁时，若处于已初始化状态，自动调用deinit（）
     */
    class MockGpio : public robot_foc::hal::Gpio
    {
    private:
        //引脚高低电平
        robot_foc::hal::GpioLevel level_ = robot_foc::hal::GpioLevel::Low;
        robot_foc::hal::GpioMode mode_ = robot_foc::hal::GpioMode::Input;

        public:
        /**
         * @brief 构造函数，传入引脚序号
         * @param pinNumber 引脚序号
         * @note explict 只能显式调用构造
         */
        explicit MockGpio(robot_foc::hal::GpioPin pin);
        ~MockGpio() noexcept override;

        /**
         * @brief 初始化、反初始化函数，模拟GPIO
         */
        bool init() noexcept override;
        bool deinit() noexcept override;

        ///@brief 引脚拉高
        [[nodiscard]] bool setHigh() noexcept override;
        ///@brief 引脚拉低
        [[nodiscard]] bool setLow() noexcept override;
        /// @brief 设置GPIO模式 输入/输出
        [[nodiscard]] bool setMode(robot_foc::hal::GpioMode mode) noexcept override;
        ///@brief 读取当前引脚电平
        [[nodiscard]] bool readLevel(robot_foc::hal::GpioLevel& out_level) const noexcept override;
    };
} // namespace robot_foc::drivers

#endif
