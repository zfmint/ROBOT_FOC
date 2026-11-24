/**
 * @file pid.cpp
 * @brief 电流环PI、位置式PID和增量式PID控制算法实现。
 * @details
 * PiController使用两阶段状态更新：preview()生成请求电压，prepare_update()依据外部限幅
 * 结果生成积分候选值，commit()由CurrentLoop在d/q两路均成功后统一执行。
 * PositionalPid采用方向感知条件积分，IncrementalPid输出控制增量。
 * @note 对外接口、错误语义和调用前置条件参见pid.hpp。
 */

#include "ALGORITHM/pid.hpp"

#include <cmath>

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
                configured_ = true;
                ki_step_ = ki_time;
                reset();
                return common::ErrorCode::Ok;
            }    
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    void PiController::reset() noexcept
    {
        integral_output_ = 0.0F;
        ++revision_;
    }

    [[nodiscard]] float PiController::PiPreview::get_request_output() const noexcept
    {
        return request_output_;
    }

    void PiController::commit(float next_integral) noexcept
    {
        integral_output_ = next_integral;
    }

    [[nodiscard]] common::ErrorCode PiController::preview(
        const float setpoint,
        const float feedback,
        PiPreview& preview_out
    ) const noexcept
    {
        preview_out.owner_ = nullptr;
        float result {0.0F};
        PiPreview out;
        float p {0.0F};
        float error {0.0F};
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if ((std::isfinite(setpoint))
         && (std::isfinite(feedback))
        )
        {
            error = setpoint - feedback;
            p = piconfig_.kp_ * error;
            result = p + integral_output_;
            if ((std::isfinite(error))
             && (std::isfinite(p))
             && (std::isfinite(result))
            )
            {
                out.owner_ = this;
                out.revision_ = revision_;
                out.error_ = error;
                out.request_output_ = result;
                preview_out = out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode PiController::prepare_update(
        const PiPreview& preview,
        float limited_output,
        bool limit_flag,
        PreparedState& next_state
    ) const noexcept
    {
        next_state.owner_ = nullptr;
        const float integral_pre = integral_output_;
        PreparedState result {};
        bool integral_worsens_saturation {false};
        bool integral_enable {true};
        if (!configured_)
        {
            return common::ErrorCode::ErrorNotReady;
        }
        if ((!std::isfinite(preview.request_output_))
         || (!std::isfinite(preview.error_))
         || (!std::isfinite(limited_output))
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        if ((preview.owner_!= this)
         || (preview.revision_ != revision_)
        )
        {
            return common::ErrorCode::ErrorUnsupported;
        }
        if (limit_flag == true)
        {
            // 若是电压进行过限幅，进行积分饱和判断。积分饱和则不允许继续积分
            integral_worsens_saturation = (((preview.request_output_ > limited_output) && (preview.error_ > 0.0F))
                                       || (((preview.request_output_ < limited_output) && (preview.error_ < 0.0F))));
            integral_enable = !integral_worsens_saturation;
        }
        if (integral_enable == true)
        {
            result.next_integral_ = integral_pre + ki_step_ * preview.error_;
        }
        else
        {
            result.next_integral_ = integral_pre;
        }
        if (!std::isfinite(result.next_integral_))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        if (result.next_integral_ < piconfig_.integral_min_)
        {
            result.next_integral_ = piconfig_.integral_min_;
        }
        else if (result.next_integral_ > piconfig_.integral_max_)
        {
            result.next_integral_ = piconfig_.integral_max_;
        }
        result.owner_ = this;
        result.revision_ = revision_;
        next_state = result;
        return common::ErrorCode::Ok;
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
