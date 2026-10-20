/**
 * @file sliding_average.hpp
 * @brief 编译期固定窗口滑动平均滤波
 * @details
 * 实现编译期固定容量环形窗口滑动平均，用于传感器浮点信号平滑。
 * 启动阶段窗口未填满时，仅对已采集有效样本计算均值，不补零；
 * 窗口满后先进先出，维护样本总和减少MCU运算开销。
 * 适用场景：电机转速、母线电压、采样电流等慢变化信号滤波。
 * 无堆内存，全部存储编译期分配；接口全部 noexcept。
 * @author Robot_FOC Project
 * @tparam Capacity 滤波窗口最大样本数量，必须 >= 2U
 * @warning 跨中断与任务并发访问，外部需要加临界区保护
 * @note 单元测试位于 Test/Src/test_sliding_average.cpp
 */

#ifndef ROBOT_FOC_ALGORITHM_SLIDING_AVERAGE_HPP
#define ROBOT_FOC_ALGORITHM_SLIDING_AVERAGE_HPP

#include <cstddef>
#include <cmath>
#include <array>
#include <type_traits>

namespace robot_foc::algorithm
{
    template <std::size_t Capacity>
    /**
     * @class SlidingAverage
     * @brief 固定窗口滑动平均滤波
     */
    class SlidingAverage
    {
        static_assert(Capacity >= 2U,"SlidingAverage: Capacity must >= 2");

    private:
        //浮点环形缓冲区存储历史采样
        std::array<float, Capacity> buf_{};
        //下一次写入位置索引
        std::size_t write_idx_{0U};
        //当前窗口有效样本计数
        std::size_t count_{0U};
        //窗口样本总和，快速计算均值
        float sum_{0.0F};

    public:
        SlidingAverage() noexcept = default;
        ~SlidingAverage() noexcept = default;

        //禁止拷贝、禁止移动
        SlidingAverage(const SlidingAverage&) = delete;
        SlidingAverage& operator=(const SlidingAverage&) = delete;
        SlidingAverage(SlidingAverage&&) = delete;
        SlidingAverage& operator=(SlidingAverage&&) = delete;

        /**
         * @brief 送入新采样点，更新滤波窗口并输出滤波结果
         * @param raw_val 原始浮点采样输入；必须是有限值（非NaN/非Inf）
         * @param [out] out 滤波后平均值；仅当返回true时有效
         * @return true  采样有效并完成滤波，out 有效
         * @return false 采样为 NaN/Inf，内部状态未改变，out 被置 0
         */
        [[nodiscard]] bool calculate(float raw_val, float& out) noexcept;

        /**
         * @brief 重置滤波状态
         * @details 清空总和、有效样本计数、写入索引
         */
        void reset() noexcept;

        /**
         * @brief 获取当前窗口内有效样本数量
         * @return size_t 当前有效样本数量
         */
        [[nodiscard]] std::size_t get_valid_count() const noexcept;

        /**
         * @brief 获取编译期配置窗口总容量
         * @return constexpr std::size_t 窗口最大可存储样本数
         */
        [[nodiscard]] constexpr std::size_t capacity() const noexcept
        {
            return Capacity;
        }
    };

    template<std::size_t Capacity>

    [[nodiscard]] bool SlidingAverage<Capacity>::calculate(float raw_val, float& out) noexcept
    {
        // 非有限输入：拒绝，保持内部状态完全不变
        if (not std::isfinite(raw_val))
        {
            out = 0.0F;
            return false;
        }

        if (count_ < Capacity)
        {
            // 启动阶段：窗口未填满，新增样本
            sum_ += raw_val;
            buf_[write_idx_] = raw_val;
            write_idx_ = (write_idx_ + 1U) % Capacity;
            ++count_;
        }
        else
        {
            // 窗口已满：移除最旧样本，加入新样本
            const float old_val = buf_[write_idx_];
            sum_ = sum_ - old_val + raw_val;
            buf_[write_idx_] = raw_val;
            write_idx_ = (write_idx_ + 1U) % Capacity;
        }
        out = sum_ / static_cast<float>(count_);
        return true;
    }

    template<std::size_t Capacity>
    void SlidingAverage<Capacity>::reset() noexcept
    {
        write_idx_ = 0U;
        count_ = 0U;
        sum_ = 0.0F;
    }

    template<std::size_t Capacity>
    [[nodiscard]] std::size_t SlidingAverage<Capacity>::get_valid_count() const noexcept
    {
        return count_;
    }

    // 编译期静态断言校验：禁止拷贝、禁止移动
    static_assert(!std::is_copy_constructible_v<SlidingAverage<4U>>);
    static_assert(!std::is_copy_assignable_v<SlidingAverage<4U>>);
    static_assert(!std::is_move_constructible_v<SlidingAverage<4U>>);
    static_assert(!std::is_move_assignable_v<SlidingAverage<4U>>);

} // namespace robot_foc::algorithm

#endif
