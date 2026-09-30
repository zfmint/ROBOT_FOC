#ifndef MOCK_GPIO_HPP
#define MOCK_GPIO_HPP

#include "HAL/peripheral.hpp"
#include <cstdint>

namespace robot_foc::drivers
{
    /**
     * @brief PC端模拟GPIO外设，用于单元测试
     * 继承Peripheral基类，不操作真实硬件寄存器
     * @note RAII约束：对象销毁时，若处于已初始化状态，自动调用deinit（）
     */
    class MockGpio : public robot_foc::hal::Peripheral
    {
    private:
        //GPIO引脚编号
        uint8_t pinNumber_;
        //引脚高低电平
        bool level_ = false;
    public:
        /**
         * @brief 构造函数，传入引脚序号
         * @param pinNumber 引脚序号
         * @note explict 只能显式调用构造
         */
        explicit MockGpio(uint8_t pinNumber);
        ~MockGpio() noexcept override;

        /**
         * @brief 初始化、反初始化函数，模拟GPIO
         */
        bool init() noexcept override;
        bool deinit() noexcept override;

        ///@brief 引脚拉高
        [[nodiscard]] bool setHigh() noexcept;
        ///@brief 引脚拉低
        [[nodiscard]] bool setLow() noexcept;
        ///@brief 读取当前引脚电平
        [[nodiscard]] bool readLevel() const noexcept;


    };
} // namespace robot_foc::drivers

#endif
