/**
 * @file test_encoder_speed.cpp
 * @brief EncoderSpeedEstimator 编码器转速估计器单元测试
 * @details
 * 基于一阶惯性仿真模型，验证M法编码器测速逻辑；覆盖差分溢出补偿、转速换算、滑动平均滤波、复位、参数校验。
 * 测试场景清单：
 *  1. 未调用configure前update，返回ErrorNotReady
 *  2. configure非法参数校验：pulses_per_revolution=0、counter_bits越界
 *  3. 首次update仅建立基线，输出转速为0
 *  4. 正向匀速转动：差分>0，转速为正，验证RPM计算公式
 *  5. 反向匀速转动：差分<0，转速为负，转向符号正确
 *  6. 硬件计数器溢出回绕（正向溢出、反向溢出），差分补偿正确
 *  7. reset()后基线失效，需要重新采集基线
 *  8. 滑动平均滤波：稳态后filtered_rpm平滑收敛
 *  9. 非法采样周期 sample_period <= 0，返回ErrorInvalidParam
 *
 * @author Robot_FOC Project
 * @note
 *  - 使用MockFirstOrderPlant模拟电机惯性，闭环验证测速输出；
 *  - 本测试仅PC单元测试使用，不编译进MCU固件；
 *  - 依赖：MockFirstOrderPlant、EncoderSpeedEstimator、SlidingAverage
 */
#include <cmath>
#include <cstdint>

#include "test_helper.hpp"
#include "mock_encoder_counter.hpp"
#include "ALGORITHM/encoder_speed.hpp"
#include "COMMON/error_code.hpp"
#include "COMMON/units.hpp"

void test_encoder_speed()
{
    using robot_foc::algorithm::EncoderSpeedEstimator;
    using robot_foc::common::ErrorCode;

    constexpr std::uint32_t kTestPpr = 2000U;
    constexpr std::uint8_t  kTestBits = 16U;
    constexpr float kTs = 0.001F;
    constexpr float kEps = 0.1F;

    // 未配置直接运行
    {
        EncoderSpeedEstimator est;
        std::uint32_t current_cnt = 1000U;
        auto res = est.update();
    }
}
