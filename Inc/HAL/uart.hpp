/**
 * @file uart.hpp
 * @brief UART HAL抽象基类，继承Peripheral，集成接收环形缓冲区
 * @details
 *  抽象UART外设，分离硬件实现与Mock仿真实现；
 *  接收采用ByteRingBuffer；
 *  接收push_rx支持两种工作模式二选一：
 *    1. 中断模式：在UART接收ISR调用push_rx
 *    2. 轮询模式：在任务循环内轮询硬件并调用push_rx
 *    约束：同一实例，**只能选择其中一种上下文作为生产者，禁止中断+任务并发调用push_rx**
 * @par 上下文约定
 *    - 工作模式二选一，不可混用：
 *      ① 中断模式：push_rx在UART接收ISR，read_rx/rx_available/rx_clear/get_rx_overflow_/clear_rx_overflow 也必须在该ISR内执行，主任务禁止调用读接口；
 *      ② 轮询模式:push_rx、read_rx及全部接收相关接口,全部在主任务执行，不使用接收中
 *    - push_rx:单生产者，禁止多个上下文并发写入
 * @par 生命周期
 *    构造 → configure(cfg) → init() → 使用外设 → deinit() → 析构
 * @warning 模板参数 RxBufferCapacity 编译期确定缓冲区大小，不可在运行时修改
 * @warning 本缓冲区无原子、无临界保护；严禁跨ISR与主任务混用读写接口，否则存在数据竞争。
 * @warning 本HAL层内置ByteRingBuffer**不提供ISR与主任务并发保护**。
      - PC单元测试环境单线程运行无问题；
      - 移植至STM32+RTOS时，ISR与任务共享访问必须增加临界区/中断屏蔽，或使用SPSC无锁环形队列。
 */
#ifndef ROBOT_FOC_HAL_UART_HPP
#define ROBOT_FOC_HAL_UART_HPP

#include <cstdint>
#include <cstddef>

#include "HAL/peripheral.hpp"
#include "HAL/byte_ring_buffer.hpp"

namespace robot_foc::hal
{    
     /// @brief UART数据位
    enum class UartDataBits : std::uint8_t
    {
        Bits8 = 8U,
        Bits7 = 7U
    };

    /// @brief UART停止位
    enum class UartStopBits : std::uint8_t
    {
        One,
        Two
    };

    /// @brief 校验位
    enum class UartParity : std::uint8_t
    {
        None,
        Even,   //偶校验
        Odd     //奇校验
    };

    /// @brief 配置存储结构体
    struct UartConfig
    {
        std::uint32_t   baudrate = 115200U;
        UartDataBits    data_bits = UartDataBits::Bits8;
        UartStopBits    stop_bits = UartStopBits::One;
        UartParity      parity = UartParity::None;
    };
    
    template<std::size_t RxBufferCapacity>

    class Uart : public Peripheral
    {
    private:
        /// @brief UART接收环形缓冲区
        ByteRingBuffer<RxBufferCapacity> rx_buf_;
        /// @brief 是否发生缓冲区满丢包标志
        bool rx_overflow_flag_{false};
        /// @brief 累计丢弃字节个数
        std::size_t rx_overflow_count_{0U};
        /// @brief 配置存储
        UartConfig config_;

    protected:
        /// @brief 传入UART配置参数
        /// @param cfg 
        void set_config(const UartConfig& cfg) noexcept
        {
            config_ = cfg;
        }

        /**
         * @brief 中断/轮询上下文调用，将硬件收到的字节压入接收环形缓冲区
         * @note 【重要】本缓冲区无原子操作、无临界区保护
         * @par 使用模式二选一，不可混用：
         *  1. 中断模式：push_rx在UART接收ISR，read_rx/rx_available/rx_clear也必须在**同一个ISR**执行；主任务禁止调用读接口
         *  2. 轮询模式：push_rx在主任务轮询，read_rx等读接口同样在主任务；全程不使用中断
         * @warning 禁止：ISR执行push_rx，同时主任务调用read_rx/rx_available/rx_clear，会产生数据竞争、未定义行为。
         * @param byte 硬件接收到的原始字节
         * @warning 同一实例只能单生产者写入；禁止多上下文并发push_rx
         */
        [[nodiscard]] bool push_rx(std::uint8_t byte) noexcept
        {
            if (!rx_buf_.push(byte))
            {
                rx_overflow_flag_ = true;
                rx_overflow_count_++;
                return false;
            }
            return true;
        }

    public:
        Uart() noexcept = default;
        ~Uart() noexcept override = default;

        /// @brief 禁止拷贝、移动
        Uart(const Uart&) = delete;
        Uart& operator=(const Uart&) = delete;
        Uart(Uart&&) noexcept = delete;
        Uart& operator=(Uart&&) noexcept = delete;

        /**
         * @brief UART外设配置，仅外设未初始化时可以调用
         * @param cfg UART参数
         * @return true 配置保存成功 false 外设已初始化，禁止修改
         */
        [[nodiscard]] bool configure(const UartConfig& cfg) noexcept
        {
            if (this->is_initialized())
            {
                return false;
            }
            if (cfg.baudrate == 0U)
            {
                return false;
            }
            config_ = cfg;
            return true;
        }

        /**
         * @brief Peripheral统一生命周期接口，无参初始化
         * @return true初始化成功；false重复初始化/配置非法
         * @note 使用configure预先设置好的UartConfig参数
         */
        [[nodiscard]] bool init() noexcept override
        {
            if (this->is_initialized())
            {
                return false;
            }
            if (config_.baudrate == 0U)
            {
                return false;
            }
            return this->mark_initialized();
        }

        /// @brief 子类必须实现deinit
        [[nodiscard]] bool deinit() noexcept override = 0;

        /**
         * @brief 获取当前生效的UART配置
         * @warning UartConfig 配置副本；对象构造后即存在默认配置，init前后均可读取
         */
        [[nodiscard]] const UartConfig& get_config() const noexcept
        {
            return config_;
        }

        /**
         * @brief 从接收缓冲区读取1字节（主循环调用）
         * @param out_byte 输出读到的字节
         * @return true 读取成功，out_byte有效，false接收缓冲区为空
         * @warning 禁止在ISR中调用
         */
        [[nodiscard]] bool read_rx(std::uint8_t& out_byte) noexcept
        {
            return rx_buf_.pop(out_byte);
        }

        /**
         * @brief 查询接收缓冲区可用的字节数量（主任务调用）
         * @return 缓冲区中待读取字节数目
         */
        [[nodiscard]] std::size_t rx_available() const noexcept
        {
            return rx_buf_.size();
        }

        /**
         * @brief 清空接收环形缓冲区（主任务调用）
         */
        void rx_clear() noexcept
        {
            rx_buf_.clear();
        }

        /**
         * @brief 查询是否发生过缓冲区溢出（主任务调用）
         * @return true 发生过 false 未发生过
         */
        [[nodiscard]] bool get_rx_overflow_flag() const noexcept
        {
            return rx_overflow_flag_;
        }

        /**
         * @brief 查询发生的丢包字节数
         * @return 一共丢失的字节数
         */
        [[nodiscard]] std::size_t get_rx_overflow_count() const noexcept
        {
            return rx_overflow_count_;
        }

        /**
         * @brief 清除溢出标记与累计计数（主任务调用）
         */
        void clear_rx_overflow() noexcept
        {
            rx_overflow_flag_ = false;
            rx_overflow_count_ = 0U;
        }

        /**
         * @brief 发送单个字节，子类硬件实现
         * @return true发送成功；false发送失败（缓冲区满/外设未初始化）
         * @note 实现语义区分：
         *  - 轮询实现：true代表字节已经硬件发送完成；
         *  - 中断发送实现：true代表字节存入发送队列，后续中断异步发送。
         */
        [[nodiscard]] virtual bool send_byte(std::uint8_t byte) noexcept = 0;

        /**
         * @brief 发送多字节数据，子类硬件实现
         * @param data 待发送数据指针，不能为nullptr
         * @param len 字节长度，0长度允许，直接返回true
         * @return true发送成功；false发送失败（发送队列满 / 外设未初始化 /空指针）
         * @note 实现语义区分：
         *  - 轮询实现：true代表全部字节已经硬件发送完成；
         *  - 中断发送实现：true代表全部字节存入发送队列，后续中断异步发送。
         */
        [[nodiscard]] virtual bool send_bytes(const std::uint8_t* data, std::size_t len) noexcept = 0;
    };
} // namespace robot_foc::hal


#endif
