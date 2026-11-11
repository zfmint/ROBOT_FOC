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
 *  3. 输出限幅仅为本环调节器输出保护，最终矢量限幅由Svpwm模块六边形限幅完成
 */
#ifndef ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP
#define ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP

#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{

    class CurrentLoop final
    {
    private:
        
    public:
        //禁止实例化
        CurrentLoop() = delete;
        ~CurrentLoop() = delete;
        CurrentLoop(const CurrentLoop&) = delete;
        CurrentLoop& operator=(const CurrentLoop&) = delete;
        CurrentLoop(CurrentLoop&&) = delete;
        CurrentLoop& operator=(CurrentLoop&&) = delete;

        [[nodiscard]] common::ErrorCode calculate(
            const common::DQCurrent& dq_input,
            const common::DQCurrent& dq_feedback,
            common::DQVoltage& dq_out
        ) noexcept;
    };
} // namespace robot_foc::algorithm

#endif
