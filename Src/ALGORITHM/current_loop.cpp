/**
 * @file current_loop.cpp
 * @brief FOC dq轴电流闭环控制器实现
 * @details
 * 内部链路：
 * 1. preview：Id/Iq预计算原始Vd/Vq
 * 2. inverse_park → VoltageLimiter矢量限幅 → park回dq
 * 3. prepare_update 根据限幅状态计算下一积分状态
 * 4. commit 固化PI积分
 * 5. 逆Park后电压送入SVPWM输出占空比
 */
#include "ALGORITHM/current_loop.hpp"

#include "units.hpp"
#include "ALGORITHM/foc_transform.hpp"
#include "ALGORITHM/voltage_limit.hpp"

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode CurrentLoop::configure(
        const PiConfig& id_config,
        const PiConfig& iq_config
    ) noexcept
    {
        if (configured_)
        {
            return common::ErrorCode::ErrorAlreadyInit;
        }
        auto ret_id = pi_id_.configure(id_config);
        if (ret_id != common::ErrorCode::Ok)
        {
            return ret_id;
        }
        auto ret_iq = pi_iq_.configure(iq_config);
        if (ret_iq != common::ErrorCode::Ok)
        {
            return ret_iq;
        }
        configured_ = true;
        return common::ErrorCode::Ok;
    }

    void CurrentLoop::reset() noexcept
    {
        pi_id_.reset();
        pi_iq_.reset();
    }

    [[nodiscard]] common::ErrorCode CurrentLoop::calculate(
        const CurrentLoopInput& current_input,
        SvpwmDuty& duty_output
    ) noexcept
    {
        const CurrentLoopInput& input = current_input;
        common::DQVoltage volt_piout;
        common::AlphaBetaVoltage volt_inversepark;
        common::DQVoltage volt_parkout;
        common::Volts id_next, iq_next;
        VoltageLimitOut limit_result;
        SvpwmDuty result;
        PiPreview id_pre, iq_pre;
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        auto ret_id = pi_id_.PiController::preview(
            input.current_reference_.d_current_,
            input.current_feedback_.d_current_,
            id_pre 
        );
        if (ret_id != common::ErrorCode::Ok)
        {
            return ret_id;
        }
        volt_piout.d_volt_= id_pre.request_output_;
        auto ret_iq = pi_iq_.PiController::preview(
            input.current_reference_.q_current_,
            input.current_feedback_.q_current_,
            iq_pre 
        );
        if (ret_iq != common::ErrorCode::Ok)
        {
            return ret_iq;
        }
        volt_piout.q_volt_ = iq_pre.request_output_;
        auto ret_inversepark = FocTransform::inverse_park(
            volt_piout,
            input.electrical_angle_,
            volt_inversepark
        );
        if (ret_inversepark != common::ErrorCode::Ok)
        {
            return ret_inversepark;
        }
        auto ret_limit = VoltageLimit::hexagon_limit(volt_inversepark,input.v_bus_,limit_result);
        if (ret_limit != common::ErrorCode::Ok)
        {
            return ret_limit;
        }
        auto ret_park = FocTransform::park(
            limit_result.alphabeta_out_,
            input.electrical_angle_,
            volt_parkout
        );
        if (ret_park != common::ErrorCode::Ok)
        {
            return ret_park;
        }
        auto ret_idupdate = pi_id_.PiController::prepare_update(
            id_pre,
            volt_parkout.d_volt_,
            limit_result.is_limited_,
            id_next
        );
        if (ret_idupdate != common::ErrorCode::Ok)
        {
            return ret_idupdate;
        }
        auto ret_iqupdate = pi_iq_.PiController::prepare_update(
            iq_pre,
            volt_parkout.q_volt_,
            limit_result.is_limited_,
            iq_next
        );
        if (ret_iqupdate != common::ErrorCode::Ok)
        {
            return ret_iqupdate;
        }
        auto ret_svpwm = Svpwm::svpwm_calculate(
            limit_result.alphabeta_out_,
            input.v_bus_,
            result
        );
        if (ret_svpwm != common::ErrorCode::Ok)
        {
            return ret_svpwm;
        }
        pi_id_.PiController::commit(id_next);
        pi_iq_.PiController::commit(iq_next);
        duty_output = result;
        return common::ErrorCode::Ok;
    }

} // namespace robot::foc::algorithm
