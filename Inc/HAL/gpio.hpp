/**
 * @file gpio.hpp
 * @brief GPIO 外设抽象接口。
 * @details
 *  用途：HAL层GPIO统一抽象，隔离上层业务与底层硬件寄存器；
 *  特性：
 *      1. 纯虚基类，定义统一GPIO操作接口，支持真实硬件GPIO与Mock模拟GPIO两套实现；
 *      2. 内置GpioMode、GpioPull、GpioLevel强类型枚举，底层固定为uint8_t，适配嵌入式寄存器；
 *      3. GpioPin强类型封装引脚号，避免裸uint8_t混淆引脚与其他数值；
 *      4. 禁止拷贝、禁止移动，外设对象不做值传递，仅传递引用；
 *      5. 全部硬件相关虚接口 noexcept，适配嵌入式环境；
 *      6. 生命周期：构造 → configure() → init() → 使用 → deinit() → 析构
 *  依赖：cstdint、HAL/peripheral.hpp，为所有GPIO外设的公共抽象层
 * @author Robot_FOC Project
 * @warning 本类为抽象基类，不可直接实例化；必须由硬件实现类或Mock模拟类继承实现
 */
#ifndef ROBOT_FOC_HAL_GPIO_HPP
#define ROBOT_FOC_HAL_GPIO_HPP

#include <type_traits>
#include <cstdint>

#include "HAL/peripheral.hpp"
#include "COMMON/error_code.hpp"

namespace robot_foc::hal
{

    /// @brief GPIO工作模式
    enum class GpioMode : std::uint8_t
    {
        Input,
        OutputPushPull,
        OutputOpenDrain
    };

    /// @brief GPIO上下拉
    enum class GpioPull : std::uint8_t
    {
        NoPull,
        PullUp,
        PullDown
    };

    /// @brief GPIO电平
    enum class GpioLevel : std::uint8_t
    {
        Low,
        High
    };

    /** @brief GPIO 引脚编号值类型，强类型防止裸数字混淆 */
    struct GpioPin
    {
        explicit constexpr GpioPin(std::uint8_t number) noexcept
            : number(number)
        {
        }

        std::uint8_t number;
    };

    /// @brief GPIO配置参数
    struct GpioConfig
    {
        GpioMode    mode{GpioMode::Input};
        GpioPull    pull{GpioPull::NoPull};
        GpioLevel   init_level{GpioLevel::Low};
    };

    /**
     * @brief GPIO 抽象接口，继承Peripheral外设生命周期
     */
    class Gpio : public Peripheral
    {
    private:
        /// @brief 引脚编号
        GpioPin     pin_;
        /// @brief GPIO配置
        GpioConfig  config_;

    protected:
        /**
         * @brief 子类内部调用，设置配置参数
         * @param cfg GPIO配置结构体
         * @return Ok 成功；ErrorAlreadyInit 外设已初始化，禁止修改配置
         */
        [[nodiscard]] ErrorCode set_config(const GpioConfig& cfg) noexcept
        {
            if (is_initialized())
            {
                return ErrorCode::ErrorAlreadyInit;
            }
            config_ = cfg;
            return ErrorCode::Ok;
        }

    public:
        explicit Gpio(GpioPin pin) noexcept;
        ~Gpio() noexcept override = default;

        /// @brief 禁止拷贝、移动
        Gpio(const Gpio&) = delete;
        Gpio& operator=(const Gpio&) = delete;
        Gpio(Gpio&&) noexcept = delete;
        Gpio& operator=(Gpio&&) noexcept = delete;

        /**
         * @brief 配置GPIO参数，仅未初始化状态可调用
         * @param cfg gpio配置
         * @return Ok；ErrorAlreadyInit；ErrorInvalidParam
         */
        [[nodiscard]] virtual ErrorCode configure(const GpioConfig& cfg) noexcept = 0;

        /**
         * @brief 初始化硬件GPIO
         * @return Ok；ErrorAlreadyInit；ErrorInvalidParam；ErrorHardwareFault
         */
        [[nodiscard]] ErrorCode init() noexcept override = 0;

        /**
         * @brief 反初始化，释放硬件相关
         * @return Ok；ErrorNotReady；ErrorHardwareFault
         */
        [[nodiscard]] ErrorCode deinit() noexcept override = 0;

        /**
         * @brief 设置输出高电平，仅输出模式有效
         * @return Ok；ErrorNotReady；ErrorUnsupported(非输出模式)
         */
        [[nodiscard]] virtual ErrorCode set_high() noexcept = 0;

        /**
         * @brief 设置输出低电平，仅输出模式有效
         * @return Ok；ErrorNotReady；ErrorUnsupported(非输出模式)
         */
        [[nodiscard]] virtual ErrorCode set_low() noexcept = 0;

        /**
         * @brief 设置GPIO工作模式
         * @param mode 目标模式
         * @return Ok；ErrorNotReady
         */
        [[nodiscard]] virtual ErrorCode set_mode(GpioMode mode) noexcept = 0;

        /**
         * @brief 读取引脚电平
         * @param out_level [out] 返回Ok时电平有效
         * @return Ok；ErrorNotReady
         */
        [[nodiscard]] virtual ErrorCode read_level(GpioLevel& out_level) const noexcept = 0;

        /**
         * @brief 获取当前GPIO配置副本
         * @return const引用GpioConfig
         */
        [[nodiscard]] const GpioConfig& get_config() const noexcept
        {
            return config_;
        }

        /**
         * @brief 获取引脚编号
         * @return GpioPin引脚对象
         */
        [[nodiscard]] GpioPin get_pin() const noexcept
        {
            return pin_;
        }
    };

    // 编译期校验Gpio：禁止拷贝、禁止移动
    static_assert(!std::is_copy_constructible_v<robot_foc::hal::Gpio>);
    static_assert(!std::is_copy_assignable_v<robot_foc::hal::Gpio>);
    static_assert(!std::is_move_constructible_v<robot_foc::hal::Gpio>);
    static_assert(!std::is_move_assignable_v<robot_foc::hal::Gpio>);

} // namespace robot_foc::hal

#endif // ROBOT_FOC_HAL_GPIO_HPP
