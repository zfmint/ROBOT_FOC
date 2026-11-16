/**
 * @file pid.cpp
 * @brief PID算法实现：PiController(分步PI)、位置式PID、增量式PID
 * @details
 *  实现 robot_foc::algorithm 命名空间下 PiController、PositionalPid 与 IncrementalPid。
 *  - PiController：分步式电流环PI控制器；preview预计算后，由update_after_limit完成条件积分与状态提交；
 *    CurrentLoop可通过私有prepare_update/commit完成d/q双轴事务更新。
 *  - PositionalPid：实现输出限幅、积分限幅，带方向感知条件积分冻结抗积分饱和逻辑。
 *  - IncrementalPid：实现增量输出限幅；输出为Δu，保存历史误差序列。
 *  约束：
 *      1. PiController：严格遵循preview → 限幅 → update_after_limit调用顺序；采样周期由PiConfig.sample_time_配置；
 *      2. PositionalPid / IncrementalPid：每次 calculate() 调用必须保持固定采样周期。
 * @author Robot_FOC Project
 * @note 接口详细文档参见 pid.hpp；本文件仅实现，不对外暴露API。
 */

#include "ALGORITHM/pid.hpp"

#include <cmath>

#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode PiController::configure(const PiConfig& config) noexcept
    {
        const PiConfig& cfg = config;
        if ((std::isfinite(cfg.kp_)&&(cfg.kp_ >= 0.0F))
         && (std::isfinite(cfg.ki_)&&(cfg.ki_ >= 0.0F))
         && (std::isfinite(cfg.sample_time_)&&(cfg.sample_time_ > 0.0F))
         && (std::isfinite(cfg.integral_max_)&&(cfg.integral_max_ >= 0.0F))
         && (std::isfinite(cfg.integral_min_)&&(cfg.integral_min_ <= 0.0F))
        )
        {
            const float ki_time = cfg.ki_ * cfg.sample_time_;
            if (std::isfinite(ki_time))
            {
                piconfig_ = cfg;
                integral_output_ = 0.0F;
                configured_ = true;
                ki_step_ = ki_time;
                return common::ErrorCode::Ok;
            }    
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    void PiController::reset() noexcept
    {
        integral_output_ = 0.0F;
    }

    [[nodiscard]] common::ErrorCode PiController::preview(
        const common::Amperes setpoint,
        const common::Amperes feedback,
        PiPreview& preview_out
    ) const noexcept
    {
        common::Volts voltage {0.0F};
        PiPreview out;
        float p {0.0F};
        float error {0.0F};
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if (std::isfinite(setpoint)&&std::isfinite(feedback))
        {
            error = setpoint - feedback;
            p = piconfig_.kp_ * error;
            voltage = p + integral_output_;
            if (std::isfinite(error)&&std::isfinite(voltage))
            {
                out.error_ = error;
                out.request_output_ = voltage;
                preview_out = out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode PiController::prepare_update(
        const PiPreview& preview,
        const common::Volts limited_output,
        const bool limited_flag,
        PiNextState& next_state_out
    ) const noexcept
    {
        const PiPreview& predata = preview;
        const float integral_pre = integral_output_;
        bool integral_worsens_saturation {false};
        bool integral_enable {true};
        PiNextState result {};
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if ((!std::isfinite(predata.request_output_))
         || (!std::isfinite(predata.error_))
         || (!std::isfinite(limited_output))
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        if (limited_flag == true)
        {
            // 若是电压进行过限幅，进行积分饱和判断。积分饱和则不允许继续积分
            integral_worsens_saturation = (((predata.request_output_ > limited_output) && (predata.error_ > 0.0F))
                                       || (((predata.request_output_ < limited_output) && (predata.error_ < 0.0F))));
            integral_enable = !integral_worsens_saturation;
        }
        if (integral_enable == true)
        {
            result.integral_out_ = integral_pre + ki_step_ * predata.error_;
        }
        else
        {
            result.integral_out_ = integral_pre;
        }
        if (result.integral_out_ < piconfig_.integral_min_)
        {
            result.integral_out_ = piconfig_.integral_min_;
        }
        else if (result.integral_out_ > piconfig_.integral_max_)
        {
            result.integral_out_ = piconfig_.integral_max_;
        }
        if (!std::isfinite(result.integral_out_))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        next_state_out = result;
        return common::ErrorCode::Ok;
    }

    common::ErrorCode PiController::commit(const PiNextState& next_state) noexcept
    {
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if ((!std::isfinite(next_state.integral_out_))
         || (next_state.integral_out_ < piconfig_.integral_min_)
         || (next_state.integral_out_ > piconfig_.integral_max_))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        integral_output_ = next_state.integral_out_;
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode PiController::update_after_limit(
        const PiPreview& preview,
        const common::Volts limited_output,
        const bool limit_flag
    ) noexcept
    {
        PiNextState next_state {};
        const common::ErrorCode prepare_result = prepare_update(
            preview,
            limited_output,
            limit_flag,
            next_state
        );
        if (prepare_result != common::ErrorCode::Ok)
        {
            return prepare_result;
        }
        return commit(next_state);
    }

    PositionalPid::PositionalPid(float kp, float ki, float kd, float out_min,
        float out_max, float integral_min, float integral_max) noexcept
        : kp_ {kp},
          ki_ {ki},
          kd_ {kd},
          out_min_ {out_min},
          out_max_ {out_max},
          integral_min_ {integral_min},
          integral_max_ {integral_max}
    {
    }

    float PositionalPid::calculate(float setpoint, float feedback) noexcept
    {
        // 计算当前误差
        const float err = setpoint - feedback;
        // 比例项
        const float p = kp_ * err;
        // 微分项（当前误差 - 上一次误差）
        const float d = kd_ * (err - last_err_);
        // 使用【旧的积分值】算出预输出（不叠加本次积分增量）
        const float pre_out = p + (ki_ * integral_) + d;
        // 抗积分饱和策略：方向感知条件积分
        // 仅当预输出饱和，且当前误差会继续加重饱和时，禁止积分累积；
        // 若饱和但误差反向，允许积分增减，实现快速退饱和；搭配积分限幅兜底。
        bool enable_integral = true;
        if (pre_out >= out_max_ && err > 0.0F)
        {
            // 正向输出饱和，误差仍为正，继续累加会加深积分饱和，禁止积分
            enable_integral = false;
        }
        else if (pre_out <= out_min_ && err < 0.0F)
        {
            // 负向输出饱和，误差仍为负，继续累加会加深积分饱和，禁止积分
            enable_integral = false;
        }
        if (enable_integral)
        {
            integral_ = integral_ + err;
        }
        //积分限幅：强制把积分项限制在上下限内，兜底保护
        if (integral_ > integral_max_)
        {
            integral_ = integral_max_;
        }
        else if (integral_ < integral_min_)
        {
            integral_ = integral_min_;
        }
        // 计算完整PID输出（使用更新后的积分）
        float out = p + ki_ * integral_ + d;
        // 输出限幅
        if (out > out_max_)
        {
            out = out_max_;
        }
        else if (out < out_min_)
        {
            out = out_min_;
        }
        last_err_ = err;
        return out;
    }

    void PositionalPid::reset() noexcept
    {
        integral_ = 0.0F;
        last_err_ = 0.0F;
    }

    void PositionalPid::set_param(float kp, float ki, float kd) noexcept
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    [[nodiscard]] float PositionalPid::get_kp() const noexcept
    {
        return kp_;
    }

    [[nodiscard]] float PositionalPid::get_ki() const noexcept
    {
        return ki_;
    }

    [[nodiscard]] float PositionalPid::get_kd() const noexcept
    {
        return kd_;
    }

    [[nodiscard]] float PositionalPid::get_integral() const noexcept
    {
        return integral_;
    }

    [[nodiscard]] float PositionalPid::get_last_err() const noexcept
    {
        return last_err_;
    }

    IncrementalPid::IncrementalPid(float kp, float ki, float kd,
        float delta_out_min, float delta_out_max) noexcept
        : kp_ {kp},
          ki_ {ki},
          kd_ {kd},
          delta_out_min_ {delta_out_min},
          delta_out_max_ {delta_out_max}
    {
    }

    float IncrementalPid::calculate(float setpoint, float feedback) noexcept
    {
        // e(k)
        err1_ = setpoint - feedback;
        // Δu
        const float delta_u = kp_ * (err1_ - err2_)
                            + ki_ * err1_
                            + kd_ * (err1_ - 2.0F * err2_ + err3_);
        // 增量限幅
        float delta_out = delta_u;
        if (delta_out > delta_out_max_)
        {
            delta_out = delta_out_max_;
        }
        else if (delta_out < delta_out_min_)
        {
            delta_out = delta_out_min_;
        }
        err3_ = err2_;
        err2_ = err1_;
        return delta_out;
    }

    void IncrementalPid::reset() noexcept
    {
        err1_ = 0.0F;
        err2_ = 0.0F;
        err3_ = 0.0F;
    }

    void IncrementalPid::set_param(float kp, float ki, float kd) noexcept
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    [[nodiscard]] float IncrementalPid::get_kp() const noexcept
    {
        return kp_;
    }

    [[nodiscard]] float IncrementalPid::get_ki() const noexcept
    {
        return ki_;
    }

    [[nodiscard]] float IncrementalPid::get_kd() const noexcept
    {
        return kd_;
    }

    [[nodiscard]] float IncrementalPid::get_err1() const noexcept
    {
        return err1_;
    }

    [[nodiscard]] float IncrementalPid::get_err2() const noexcept
    {
        return err2_;
    }

    [[nodiscard]] float IncrementalPid::get_err3() const noexcept
    {
        return err3_;
    }

} // namespace robot_foc::algorithm
