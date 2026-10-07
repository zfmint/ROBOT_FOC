/**
 * @file mock_uart.hpp
 * @brief UART mock仿真实现，PC单元测试
 */

#ifndef ROBOT_FOC_DRIVERS_MOCK_UART_HPP
#define ROBOT_FOC_DRIVERS_MOCK_UART_HPP

#include <cstdint>
#include <cstddef>

#include "HAL/uart.hpp"
#include "HAL/byte_ring_buffer.hpp"

namespace robot_foc::drivers
{
    template<std::size_t RxBufferCapacity,
             std::size_t TxBufferCapacity>

    class MockUart final : public robot_foc::hal::Uart<RxBufferCapacity>
    {
    private:
        /// @brief 发送缓冲区
        robot_foc::hal::ByteRingBuffer<TxBufferCapacity> tx_buf_;
    public:
        MockUart() noexcept = default;
        ~MockUart() noexcept override
        {
            if (this->is_initialized())
            {
                (void) deinit();
            }
            
        }

        MockUart(const MockUart&) = delete;
        MockUart& operator=(const MockUart&) = delete;
        MockUart(MockUart&&) noexcept = delete;
        MockUart& operator=(MockUart&&) noexcept = delete;
        
        /// @brief 反初始化
        /// @return true 反初始化成功
        [[nodiscard]] bool deinit() noexcept override
        {
            if (!this->is_initialized())
            {
                return false;
            }
            (void) tx_clear();
            (void) this->rx_clear();
            this->clear_rx_overflow();
            robot_foc::hal::UartConfig default_cfg{};
            this->set_config(default_cfg);
            (void) this->mark_deinitialized();
            return true;
        }

        /**
         * @brief 发送单个字节数据
         */
        [[nodiscard]] bool send_byte(std::uint8_t byte) noexcept override
        {
            if (!this->is_initialized())
            {
                return false;
            }
            return tx_buf_.push(byte);
        }

        /**
         * @brief 发送多个字节数据
         */
        [[nodiscard]] bool send_bytes(const std::uint8_t* data, std::size_t len) noexcept override
        {
            ///未初始化
            if (!this->is_initialized())
            {
                return false;
            }
            ///0长度返回true
            if (len == 0U)
            {
                return true;
            }
            ///空指针校验
            if (data == nullptr)
            {
                return false;
            }
            ///预检查，缓冲区剩余空间是否足够
            if (tx_buf_.capacity() - tx_buf_.size() < len)
            {
                return false;
            }
            ///空间足够，一次性全部写入
            for (std::size_t i = 0U; i < len; ++i)
            {
                (void) tx_buf_.push(data[i]);
            }
            return true;
        }

        /// @brief 测试用tx接口        
        /**
         * @brief 清空发送环形缓冲区（主任务调用）
         */
        void tx_clear() noexcept
        {
            tx_buf_.clear();
        }

        /**
         * @brief 查询发送缓冲区可用的字节数量（主任务调用）
         * @return 缓冲区中待发送字节数目
         */
        [[nodiscard]] std::size_t tx_available() const noexcept
        {
            return tx_buf_.size();
        }

        /**
         * @brief 从发送缓冲区读取1字节（主循环调用）
         */
        [[nodiscard]] bool tx_read(std::uint8_t& out_byte) noexcept
        {
            return tx_buf_.pop(out_byte);
        }

        /**
         * @brief 模拟硬件收到一字节，注入到Uart基类接收缓冲区
         * @param byte 模拟收到的串口字节
         * @return true 注入成功 false 外设未初始化，拒绝写入
         * @note 单元测试调用，等价于硬件ISR里调用push_rx
         */
        [[nodiscard]] bool inject_rx(std::uint8_t byte) noexcept
        {
            if (!this->is_initialized())
            {
                return false;
            }
            return this->push_rx(byte);
        }
    };
} // namespace robot_foc::drivers


#endif
