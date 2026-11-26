/**
 * @file current_loop.cpp
 * @brief FOC dq轴电流闭环控制器实现
 * @details
 * 内部链路：
 * 1. preview：Id/Iq预计算原始Vd/Vq
 * 2. inverse_park → VoltageLimiter矢量限幅 → park回dq
 * 3. prepare_update 根据限幅状态计算下一积分状态
 * 4. 限幅后αβ电压送入SVPWM计算占空比
 * 5. 全部计算成功后commit固化PI积分并提交占空比
 */
#include "ALGORITHM/current_loop.hpp"

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

        const auto ret_id = pi_id_.configure(id_config);
        if (ret_id != common::ErrorCode::Ok)
        {
            return ret_id;
        }

        const auto ret_iq = pi_iq_.configure(iq_config);
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
        CurrentLoopOutput& current_output
    ) noexcept
    {
        const CurrentLoopInput& input = current_input;
        common::DQVoltage volt_piout {};
        common::AlphaBetaVoltage volt_inversepark {};
        common::DQVoltage volt_parkout {};
        PiController::PreparedState id_next {};
        PiController::PreparedState iq_next {};
        VoltageLimitOut limit_result {};
        CurrentLoopOutput result {};
        PiController::PiPreview id_pre {};
        PiController::PiPreview iq_pre {};
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }

        const auto ret_id = pi_id_.preview(
            input.current_reference_.d_current_,
            input.current_feedback_.d_current_,
            id_pre 
        );
        if (ret_id != common::ErrorCode::Ok)
        {
            return ret_id;
        }
        volt_piout.d_volt_ = id_pre.get_request_output();
        const auto ret_iq = pi_iq_.preview(
            input.current_reference_.q_current_,
            input.current_feedback_.q_current_,
            iq_pre 
        );
        if (ret_iq != common::ErrorCode::Ok)
        {
            return ret_iq;
        }
        volt_piout.q_volt_ = iq_pre.get_request_output();

        const auto ret_inversepark = FocTransform::inverse_park(
            volt_piout,
            input.electrical_angle_,
            volt_inversepark
        );
        if (ret_inversepark != common::ErrorCode::Ok)
        {
            return ret_inversepark;
        }

        const auto ret_limit = VoltageLimit::hexagon_limit(
            volt_inversepark,
            input.v_bus_,
            limit_result
        );
        if (ret_limit != common::ErrorCode::Ok)
        {
            return ret_limit;
        }

        const auto ret_park = FocTransform::park(
            limit_result.alphabeta_out_,
            input.electrical_angle_,
            volt_parkout
        );
        if (ret_park != common::ErrorCode::Ok)
        {
            return ret_park;
        }

        const auto ret_idupdate = pi_id_.prepare_update(
            id_pre,
            volt_parkout.d_volt_,
            limit_result.is_limited_,
            id_next
        );
        if (ret_idupdate != common::ErrorCode::Ok)
        {
            return ret_idupdate;
        }
        const auto ret_iqupdate = pi_iq_.prepare_update(
            iq_pre,
            volt_parkout.q_volt_,
            limit_result.is_limited_,
            iq_next
        );
        if (ret_iqupdate != common::ErrorCode::Ok)
        {
            return ret_iqupdate;
        }

        const auto ret_svpwm = Svpwm::svpwm_calculate(
            limit_result.alphabeta_out_,
            input.v_bus_,
            result.duty_
        );
        if (ret_svpwm != common::ErrorCode::Ok)
        {
            return ret_svpwm;
        }

        const auto ret_commit = PiController::commit_pair(pi_id_, id_next, pi_iq_, iq_next);
        if (ret_commit != common::ErrorCode::Ok)
        {
            return ret_commit;
        }
        result.is_voltage_limited_ = limit_result.is_limited_;
        current_output = result;
        return common::ErrorCode::Ok;
    }
} // namespace robot_foc::algorithm
