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
        Peripheral(Peripheral&&) noexcept = default;
        //移动赋值（标准写法）转移所有权
        Peripheral& operator = (Peripheral&&) noexcept = default;

        /**
         * @brief 外设初始化，子类实现
         * @return true 初始化成功，false 初始化失败
        */
        virtual bool init() = 0;
        /**
         * @brief 外设反初始化，释放硬件资源，子类实现
         * @return true 反初始化成功，false 反初始化失败
        */
        virtual bool deinit() = 0;

       /**
         * @brief 读取初始化状态，const只读，无法修改
         * @return true 已初始化，false 未初始化
        */
        bool is_inited() const;
    protected:
        //保护变量，记录初始化状态，但怎么调用？
        bool m_inited = false;

    };
}

#endif
