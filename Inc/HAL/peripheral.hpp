/**
 * @file peripheral.hpp
 * @brief 外设顶层抽象基类，HAL层所有外设的统一父类
 *
 * 设计层级关系：
 * Peripheral（顶层外设生命周期基类，本文件）
 *     └── Gpio（GPIO外设抽象接口）
 *             └── MockGpio（PC仿真GPIO实现，单元测试用）
 *             └── Stm32Gpio（单片机真实硬件GPIO实现，后续开发）
 *     └── Uart（串口抽象接口，待实现）
 *     └── Spi（SPI抽象接口，待实现）
 *
 * 职责：
 * 1. 统一管理外设生命周期状态：维护initialized_初始化标记
 * 2. 提供统一生命周期接口：init() / deinit()，由子类实现硬件逻辑
 * 3. 提供状态访问接口 is_initialized()、状态标记 mark_initialized / mark_deinitialized
 * 4. 约束对象语义：禁止拷贝、禁止移动，外设对象生命周期固定，不允许转移所有权
 * 5. 无异常设计，全部接口 noexcept，使用bool返回值表达执行成功/失败，不使用C++异常
 *
 * 设计目标：
 * 所有硬件外设驱动继承该基类，对外提供统一的生命周期API；
 * 实现PC端Mock仿真版本与嵌入式硬件版本的接口对齐，一套上层业务代码可同时跑仿真与硬件。
 */
#ifndef PERIPHERAL_HPP
#define PERIPHERAL_HPP

namespace robot_foc::hal{
    /**
     * @brief 外设顶层抽象类
     * 所有硬件外设驱动继承此类，统一提供init与deinit生命周期接口
     * 禁止拷贝
     * 支持移动语义
     * 禁用异常
     * 使用bool返回值表示执行结果
     */
    class Peripheral{
    private:
        //私有变量，记录初始化状态
        bool initialized_ = false;    
    protected:
        //[[nodiscard]] 表示属性
        //以下表示必须接收返回值，不能直接丢弃返回的bool
        [[nodiscard]] bool mark_initialized() noexcept;
        [[nodiscard]] bool mark_deinitialized() noexcept;
    public:
        //默认构建函数
        Peripheral() = default;
        //虚拟析构函数，类设计用于被继承，基类析构必须使用virtual
        virtual ~Peripheral() = default;

        //禁止拷贝构造（标准写法）
        //=delete 表示删除这个函数
        Peripheral(const Peripheral&) = delete;
        //禁止拷贝赋值（标准写法）
        Peripheral& operator = (const Peripheral&) = delete;

        //移动构造（标准写法） noexcept表示无异常抛出，嵌入式极少使用异常
        Peripheral(Peripheral&&) noexcept = delete;
        //移动赋值（标准写法）转移所有权
        Peripheral& operator = (Peripheral&&) noexcept = delete;

        /**
         * @brief 外设初始化，子类实现
         * @return true 初始化成功，false 初始化失败
        */
        virtual bool init() noexcept = 0;
        /**
         * @brief 外设反初始化，释放硬件资源，子类实现
         * @return true 反初始化成功，false 反初始化失败
        */
        virtual bool deinit() noexcept = 0;

       /**
         * @brief 读取初始化状态，const只读，无法修改
         * @return true 已初始化，false 未初始化
        */
        bool is_initialized() const noexcept;
    };
}

#endif
