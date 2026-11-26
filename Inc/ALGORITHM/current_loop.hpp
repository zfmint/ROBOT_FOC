/**
 * @file current_loop.hpp
 * @brief FOC dq轴电流闭环控制器（电流内环）
 * @details
 * 接收目标Id/Iq、反馈Id/Iq，通过独立分步PI调节器计算输出Vd、Vq电压指令；
 * 内部集成：PI分步运算、逆Park、电压矢量限幅、SVPWM，对外直接输出占空比；
 * 纯算法，无硬件依赖，支持PC单元测试。
 * @note
 *  1. 电流环运行频率一般与PWM载波频率保持一致
 *  2. Id为励磁电流环、Iq为转矩电流环，两组调节器参数独立配置
 *  3. 积分状态由PiController内部维护；矢量限幅由 VoltageLimit 完成
 *  4. 使用分步式PiController：preview‑prepare_update‑commit
 *  5. 本类为最终实现类，禁止继承
 */
#ifndef ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP
#define ROBOT_FOC_ALGORITHM_CURRENT_LOOP_HPP

#include "error_code.hpp"
#include "control_type.hpp"
#include "ALGORITHM/pid.hpp"
#include "ALGORITHM/svpwm.hpp"

namespace robot_foc::algorithm
{
    struct CurrentLoopInput
    {
        common::DQCurrent current_reference_ {};
        common::DQCurrent current_feedback_ {};
        common::ElectricalRad electrical_angle_ {0.0F};
        common::Volts v_bus_ {0.0F};
    };

    struct CurrentLoopOutput
    {
        SvpwmDuty duty_ {};
        bool is_voltage_limited_ {false};
    };

    class CurrentLoop final
    {
    private:
        PiController pi_id_;
        PiController pi_iq_;
        bool configured_ {false};
    public:
        /**
         * @brief 配置Id/Iq两路PI参数
         * @param[in] id_config d轴电流PI配置参数
         * @param[in] iq_config q轴电流PI配置参数
         * @retval Ok 配置成功
         * @retval ErrorInvalidParam 参数非法
         * @retval ErrorAlreadyInit CurrentLoop已经成功配置
         */
        [[nodiscard]] common::ErrorCode configure(
            const PiConfig& id_config,
            const PiConfig& iq_config
        ) noexcept;

        /**
         * @brief 执行完整电流环计算链路
         * @param current_input 参考电流、反馈电流、电角度、母线电压
         * @param current_output 输出三相SVPWM占空比、电压限幅标志
         * @retval Ok 正常完成计算
         * @retval ErrorNotReady 未完成configure配置
         * @retval ErrorInvalidParam 输入存在NaN/Inf非法数值
         * @note 下游算法模块发生错误时，原样返回对应错误码。
         * @note 仅当返回Ok时，current_output才更新为本周期有效结果；
         *       计算失败时，current_output保持调用前的状态。
         */
        [[nodiscard]] common::ErrorCode calculate(
            const CurrentLoopInput& current_input,
            CurrentLoopOutput& current_output
        ) noexcept;

        /**
         * @brief 复位两路PI积分状态，PI增益配置保持不变
         */
        void reset() noexcept;
    };
} // namespace robot_foc::algorithm
#endif
