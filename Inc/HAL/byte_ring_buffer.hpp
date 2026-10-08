/**
 * @file byte_ring_buffer.hpp
 * @brief 编译期固定容量 字节环形缓冲区
 * @details
 *  用途：UART接收缓存等场景，存放uint8_t原始字节；
 *  特性：
 *      1. 模板参数Capacity：编译期指定缓冲区总大小，无堆动态内存；
 *      2. 使用独立count_成员记录有效字节数
 *         empty：count_ = 0
 *         full：count_ = Capacity
 *      3. push缓冲区满返回false，**绝不覆盖旧数据**；
 *      4. 全部接口 noexcept，适配嵌入式环境；
 *      5. 禁止拷贝、禁止移动，避免栈数组拷贝造成严重bug；
 *  依赖：仅cstddef cstdint，不依赖FreeRTOS，不依赖其他业务组件
 * @author Robot_FOC Project
 * @warning 若跨中断/主循环并发访问本缓冲区，push/pop/peek/empty/full/size/clear 必须使用中断临界区保护
 */

#ifndef ROBOT_FOC_HAL_BYTE_RING_BUFFER_HPP
#define ROBOT_FOC_HAL_BYTE_RING_BUFFER_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace robot_foc::hal
{
    template<std::size_t Capacity>

    class ByteRingBuffer
    {
    // 模板参数必须 >= 2，可以存 2 字节
    static_assert(Capacity >= 2U, "ByteRingBuffer: Capacity must >= 2");

    private:
        /// @brief 底层存储数组
        std::uint8_t buf_[Capacity]{};
        /// @brief 下一次写入的位置下标
        std::size_t write_idx_{};
        /// @brief 下一次读取的位置下标
        std::size_t read_idx_{};
        /// @brief 当前缓冲区有效字节数
        std::size_t count_{0U};

    public:
        ByteRingBuffer() noexcept = default;
        ~ByteRingBuffer() noexcept = default;

        /**
         * @brief 禁止拷贝、禁止移动
         * @note 容器内部包含栈数组buf_[Capacity]；Capacity可配置为较大值。
         * @note 禁止拷贝目的：防止无意的值传递，造成栈上复制大块缓冲区内存。
         * @note 使用范式：仅作为类内嵌成员或者局部栈对象；对外传递使用引用。
         */
        ByteRingBuffer(const ByteRingBuffer&) = delete;
        ByteRingBuffer& operator=(const ByteRingBuffer&) = delete;
        ByteRingBuffer(ByteRingBuffer&&) = delete;
        ByteRingBuffer& operator=(ByteRingBuffer&&) = delete;

        /**
         * @brief 判断缓冲区是否为空
         * @return true 为空
         */
        [[nodiscard]] bool empty() const noexcept
        {
            return count_ == 0U;
        }

        /**
         * @brief 判断缓冲区是否为满
         * @return true 为满,无法写入
         */
        [[nodiscard]] bool full() const noexcept
        {
            return count_ == Capacity;
        }

        /**
         * @brief 获取缓冲区总容量（编译期常量）
         * @return 总字节数Capacity，最多可存入Capacity字节
         */
        [[nodiscard]] constexpr std::size_t capacity() const noexcept
        {
            return Capacity;
        }

        /**
         * @brief 获取当前有效字节数量
         * @return 当前存储的有效字节数目
         */
        [[nodiscard]] std::size_t size() const noexcept
        {
            return count_;
        }

        /**
         * @brief 向缓冲区尾部压入一个字节
         * @param byte 待写入字节
         * @return true 写入成功，false 写入失败
         */
        [[nodiscard]] bool push(std::uint8_t byte) noexcept
        {
            if (full())
            {
                return false;
            }
            buf_[write_idx_] = byte;
            // 写指针前进，取模环绕
            write_idx_ = (write_idx_ + 1U) % Capacity;
            ++count_;
            return true;
        }

        /**
         * @brief 缓冲区头部弹出一个字节
         * @param out_byte [out] 输出读到的字节，仅返回true时输出有效
         * @return true 读取成功，false 缓冲区为空，读取失败
         */
        [[nodiscard]] bool pop(std::uint8_t& out_byte) noexcept
        {
            if (empty())
            {
                return false;
            }
            out_byte = buf_[read_idx_];
            // 读指针前进，取模环绕
            read_idx_ = (read_idx_ + 1U) % Capacity;
            --count_;
            return true;
        }

        /**
         * @brief 查看缓冲区首字节，不消费，不移除数据
         * @param out_byte [out] 输出队首字节
         * @return true 获取成功，false 缓冲区为空
         */
        [[nodiscard]] bool peek(std::uint8_t& out_byte) const noexcept
        {
            if (empty())
            {
                return false;
            }
            out_byte = buf_[read_idx_];
            return true;
        }

        /**
         * @brief 清空缓冲区，恢复为空状态
         * @note 仅重置读写下标与计数，不会清零buf_数组；
         */
        void clear() noexcept
        {
            write_idx_ = 0U;
            read_idx_ = 0U;
            count_ = 0U;
        }
    };
    
    // 编译期校验 ByteRingBuffer<8U>：禁止拷贝、禁止移动
    static_assert(!std::is_copy_constructible_v<robot_foc::hal::ByteRingBuffer<8U>>);
    static_assert(!std::is_copy_assignable_v<robot_foc::hal::ByteRingBuffer<8U>>);
    static_assert(!std::is_move_constructible_v<robot_foc::hal::ByteRingBuffer<8U>>);
    static_assert(!std::is_move_assignable_v<robot_foc::hal::ByteRingBuffer<8U>>);
} // namespace robot_foc::hal

#endif // ROBOT_FOC_HAL_BYTE_RING_BUFFER_HPP
