/**
 * @file mock_uart.hpp
 * @brief UART mock仿真实现，PC单元测试
 */

#ifndef ROBOT_FOC_DRIVERS_MOCK_UART_HPP
#define ROBOT_FOC_DRIVERS_MOCK_UART_HPP

#include <type_traits>
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
        typename robot_foc::hal::ByteRingBuffer<TxBufferCapacity> tx_buf_;
        using ErrorCode = typename robot_foc::hal::Uart<RxBufferCapacity>::ErrorCode;
    public:
        MockUart() noexcept = default;
        ~MockUart() noexcept override = default;

        MockUart(const MockUart&) = delete;
        MockUart& operator=(const MockUart&) = delete;
        MockUart(MockUart&&) noexcept = delete;
        MockUart& operator=(MockUart&&) noexcept = delete;

        /**
         * @brief 初始化Mock UART
         * @return Ok；ErrorAlreadyInit
         */
        [[nodiscard]] ErrorCode init() noexcept override
        {
            if (this->is_initialized())
            {
                return ErrorCode::ErrorAlreadyInit;
            }
            const auto& cfg = this->get_config();
            if (!this->validate_config(cfg))
            {
                return ErrorCode::ErrorInvalidParam;
            }
            return this->mark_initialized();
        }

        /**
         * @brief 反初始化
         * @return Ok；ErrorNotReady
         */
        [[nodiscard]] ErrorCode deinit() noexcept override
        {
            if (!this->is_initialized())
            {
                return ErrorCode::ErrorNotReady;
            }
            auto ret_mark = this->mark_deinitialized();
            if (ret_mark != ErrorCode::Ok)
            {
                return ret_mark;
            }
            
            tx_clear();
            this->rx_clear();
            this->clear_rx_overflow();
            
            return ErrorCode::Ok;
        }

        /**
         * @brief 发送单个字节数据
         * @return Ok；ErrorNotReady；ErrorBufferFull
         */
        [[nodiscard]] ErrorCode send_byte(std::uint8_t byte) noexcept override
        {
            if (!this->is_initialized())
            {
                return ErrorCode::ErrorNotReady;
            }
            if (tx_buf_.push(byte))
            {
                return ErrorCode::Ok;
            }
            return ErrorCode::ErrorBufferFull;
        }

        /**
         * @brief 发送多个字节数据
         * @return Ok；ErrorNotReady；ErrorBufferFull；ErrorInvalidParam
         */
        [[nodiscard]] ErrorCode send_bytes(const std::uint8_t* data, std::size_t len) noexcept override
        {
            // 未初始化
            if (!this->is_initialized())
            {
                return ErrorCode::ErrorNotReady;
            }
            // 0长度直接成功
            if (len == 0U)
            {
                return ErrorCode::Ok;
            }
            // 空指针校验
            if (data == nullptr)
            {
                return ErrorCode::ErrorInvalidParam;
            }
            // 预检查，缓冲区剩余空间是否足够
            if (tx_buf_.capacity() - tx_buf_.size() < len)
            {
                return ErrorCode::ErrorBufferFull;
            }
            // 空间足够，一次性全部写入
            for (std::size_t i = 0U; i < len; ++i)
            {
                (void) tx_buf_.push(data[i]);
            }
            return ErrorCode::Ok;
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
         * @brief 查询发送缓冲区待发送字节数量（主任务调用）
         * @return 缓冲区中待发送字节数目
         */
        [[nodiscard]] std::size_t tx_available() const noexcept
        {
            return tx_buf_.size();
        }

        /**
         * @brief 从发送缓冲区读取1字节（单元测试使用，取出mock收到的发送数据）
         * @param out_byte [out] 输出读到的字节
         * @return true读取成功，false缓冲区空
         */
        [[nodiscard]] bool tx_read(std::uint8_t& out_byte) noexcept
        {
            return tx_buf_.pop(out_byte);
        }

        /**
         * @brief 模拟硬件收到一字节，注入到Uart基类接收缓冲区
         * @param byte 模拟收到的串口字节
         * @return Ok成功；ErrorNotReady外设未初始化；ErrorBufferFull接收缓存满
         * @note 单元测试调用，等价于硬件ISR里调用push_rx
         */
        [[nodiscard]] ErrorCode inject_rx(std::uint8_t byte) noexcept
        {
            if (!this->is_initialized())
            {
                return ErrorCode::ErrorNotReady;
            }
            return this->push_rx(byte);
        }
    };

    // 编译期校验 MockUart：禁止拷贝、禁止移动
    static_assert(!std::is_copy_constructible_v<robot_foc::drivers::MockUart<8U, 8U>>);
    static_assert(!std::is_copy_assignable_v<robot_foc::drivers::MockUart<8U, 8U>>);
    static_assert(!std::is_move_constructible_v<robot_foc::drivers::MockUart<8U, 8U>>);
    static_assert(!std::is_move_assignable_v<robot_foc::drivers::MockUart<8U, 8U>>);

} // namespace robot_foc::drivers

#endif // ROBOT_FOC_DRIVERS_MOCK_UART_HPP
