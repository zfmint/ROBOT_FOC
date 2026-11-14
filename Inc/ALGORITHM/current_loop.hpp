/**
 * @file current_loop.hpp
 * @brief FOC dq轴电流闭环控制器（电流内环）
 * @details
 * 接收目标Id/Iq、反馈Id/Iq，通过独立PI调节器计算输出Vd、Vq电压指令；
 * 仅做控制算法运算，不包含坐标变换、SVPWM、硬件采样逻辑；
 * 内部维护两套PI调节器状态，支持积分清零、动态配置更新；纯算法，无硬件依赖，支持PC单元测试。
 * @note
 *  1. 电流环运行频率一般与PWM载波频率保持一致
 *  2. Id为励磁电流环、Iq为转矩电流环，两组调节器参数独立配置
 *  3. 本环调节器输出仅做内部保护限幅；最终αβ电压矢量限幅由 VoltageLimiter 模块完成
 *  4. 使用位置式PID
 *  5. 本类为最终实现类，禁止继承
 */
#ifndef ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP
#define ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"
#include "ALGORITHM/pid.hpp"
#include "ALGORITHM/foc_transform.hpp"

namespace robot_foc::algorithm
{
    class CurrentLoop final
    {
    private:
        PositionalPid id_cfg;
        PositionalPid iq_cfg;
    public:
        [[nodiscard]] common::ErrorCode calculate(
            const common::DQCurrent& dq_ref,
            const common::DQCurrent& dq_feedback,
            common::DQVoltage& dq_out
        ) noexcept;
    };
} // namespace robot_foc::algorithm

#endif
