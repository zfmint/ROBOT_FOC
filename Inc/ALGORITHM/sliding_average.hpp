/**
 * @file sliding_average.hpp
 * @brief 固定窗口滑动平均滤波
 * @details
 * 实现固定容量环形窗口滑动平均，用于传感器信号平滑。
 * 启动阶段窗口未填满时，仅对已采集有效样本计算均值，不补零；
 * 窗口满后采用先进先出，维护样本总和减少MCU运算开销。
 * 适用场景：电机转速、母线电压、采样电流等慢变化信号滤波。
 * @author Robot_FOC Project
 * @note
 *  1. 窗口容量在构造时指定；
 *  2. 调用reset()清空缓存与状态；
 *  3. 单元测试位于Test/Src/test_sliding_average.cpp
 */

#ifndef ROBOT_FOC_ALGORITHM_SLIDING_AVERAGE_HPP
#define ROBOT_FOC_ALGORITHM_SLIDING_AVERAGE_HPP

#include <cstddef>
#include <cmath>

namespace robot_foc::algorithm
{
    /**
     * @class SlidingAverage
     * @brief 固定窗口滑动平均滤波
     */
    class SlidingAverage
    {
    private:
        /* data */
    public:
        /**
        * @param window_size 滤波窗口最大样本容量
        */
        explicit SlidingAverage(std::size_t window_size) noexcept;

        [[]] float 
    };
    
} // namespace robot_foc::algorithm




#endif
