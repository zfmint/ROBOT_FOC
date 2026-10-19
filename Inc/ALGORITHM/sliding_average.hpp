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
        std::size_t write_index_;
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
         * @brief 送入新采样点
         * @param raw_val 原始采样值
         * @return float 滑动滤波平均输出
         */
        [[nodiscard]] float calculate(float raw_val) noexcept;

        /**
         * @brief 重置滤波状态
         * @details 清空缓冲区、总和、有效样本计数、写入索引
         */
        void reset() noexcept;

        /**
         * @brief 获取当前窗口内有效样本数量
         * @return size_t 当前有效样本数量
         */
        [[nodiscard]] std::size_t get_vailid_count() const noexcept;

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
    [[nodiscard]] float SlidingAverage<Capacity>::calculate(float raw_val) noexcept
    {
        if (count_ < Capacity)
        {
            //启动阶段：窗口未填满，新增样本
            sum_ += raw_val;
            buf_[write_index_] = raw_val;
            write_index_ = (write_index_ + 1U) % Capacity;
            ++count_;
        }
        
    }
    
} // namespace robot_foc::algorithm




#endif
