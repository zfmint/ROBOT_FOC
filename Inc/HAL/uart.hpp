/**
 * @file uart.hpp
 * @brief UART HAL抽象基类，继承Peripheral，集成接收环形缓冲区
 * @details
 *  抽象UART外设，分离硬件实现与Mock仿真实现；
 *  接收采用ByteRingBuffer；
 * @par 上下文约定
 *    - isr_push_rx: 仅可在UART接收中断服务函数调用
 *    - read_rx / rx_available / clear_rx / get_rx_overflow_*: 仅主任务调用，禁止ISR调用
 * @par 生命周期
 *    构造 → init() → 使用外设 → deinit() → 析构
 * @warning 模板参数 RxBufferCapacity 编译期确定缓冲区大小，不可在运行时修改
 */
 #ifndef ROBOT_FOC_HAL_UART_HPP
 #define ROBOT_FOC_HAL_UART_HPP

#include "HAL/peripheral.hpp"
#include "HAL/byte_ring_buffer.hpp"
#include <cstdint>

namespace robot_foc::hal
{
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

    public:
        Uart() noexcept = default;
        ~Uart() noexcept override = default;

        /// @brief 禁止拷贝、移动
        Uart(const Uart&) = delete;
        Uart& operator=(const Uart&) = delete;
        Uart(Uart&&) noexcept = delete;
        Uart& operator=(Uart&&) noexcept = delete;

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
         * @brief 中断上下文调用，将硬件收到的字节压入接收环形缓冲区
         * @note 仅允许单生产者上下文调用：要么UART接收中断，要么轮询任务；
         * @param byte 硬件接收到的原始字节
         * @warning 禁止同时从中断与任务并发调用，内部无互斥保护；仅内存操作，不访问硬件
         */
        void push_rx(std::uint8_t byte) noexcept
        {
            if (!rx_buf_.push(byte))
            {
                rx_overflow_flag_ = true;
                rx_overflow_count_++;
            }
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
