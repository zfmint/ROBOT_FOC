/**
 * @file peripheral.hpp
 * @brief HAL 外设顶层抽象基类。
 * @details
 *  用途：所有硬件外设的统一生命周期抽象，统一管理init/deinit初始化状态；
 *  特性：
 *      1. 纯虚基类，定义外设标准生命周期接口：init / deinit；
 *      2. 内置initialized_私有状态标记，提供is_initialized查询、mark_initialized/mark_deinitialized状态修改；
 *      3. 禁止拷贝、禁止移动，外设对象绑定硬件资源，不允许隐式所有权转移；
 *      4. 析构函数虚且noexcept，支持派生类安全析构；
 *      5. 全部接口noexcept，适配嵌入式环境；
 *  依赖：无额外头文件依赖，是HAL层所有外设（GPIO、UART等）的顶层父类
 * @author Robot_FOC Project
 * @warning 本类为抽象基类，不能直接实例化；必须由具体外设类继承实现init/deinit虚函数
 * @note 状态标记由基类内部维护，派生类仅可通过protected辅助函数修改状态，禁止直接读写initialized_
 */
#ifndef ROBOT_FOC_HAL_PERIPHERAL_HPP
#define ROBOT_FOC_HAL_PERIPHERAL_HPP

namespace robot_foc::hal
{

    /**
     * @brief 所有硬件外设的统一生命周期接口。
     * 外设对象禁止拷贝和移动，避免硬件资源所有权被隐式转移。
     */
    class Peripheral
    {
    private:
        ///@brief 初始化完成标志
        bool initialized_{false};

    public:
        Peripheral() noexcept = default;
        virtual ~Peripheral() noexcept = default;

        Peripheral(const Peripheral&) = delete;
        Peripheral& operator=(const Peripheral&) = delete;
        Peripheral(Peripheral&&) noexcept = delete;
        Peripheral& operator=(Peripheral&&) noexcept = delete;

        /**
         * @brief 初始化外设资源。
         * @return true 表示初始化成功，false 表示失败或已初始化。
         */
        [[nodiscard]] virtual bool init() noexcept = 0;

        /**
         * @brief 释放外设资源。
         * @return true 表示反初始化成功，false 表示失败或未初始化。
         */
        [[nodiscard]] virtual bool deinit() noexcept = 0;

        /** @brief 查询外设是否已经初始化。 */
        [[nodiscard]] bool is_initialized() const noexcept;

    protected:
        /** @brief 将状态从未初始化转换为已初始化。 */
        [[nodiscard]] bool mark_initialized() noexcept;

        /** @brief 将状态从已初始化转换为未初始化。 */
        [[nodiscard]] bool mark_deinitialized() noexcept;
    };

} // namespace robot_foc::hal

#endif // PERIPHERAL_HPP
