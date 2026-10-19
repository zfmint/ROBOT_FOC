/**
 * @file pid.cpp
 * @brief PID算法实现：位置式PID、增量式PID
 * @details
 *  实现 robot_foc::algorithm 命名空间下 PositionalPid 与 IncrementalPid。
 *  - PositionalPid：实现输出限幅、积分限幅，带抗积分饱和逻辑。
 *  - IncrementalPid：实现增量输出限幅；输出为Δu。
 *  约束：每次 calculate() 调用必须保持固定采样周期。
 * @author Robot_FOC Project
 * @note 接口详细文档参见 pid.hpp；本文件仅实现，不对外暴露API。
 */

#include "ALGORITHM/pid.hpp"

namespace robot_foc::algorithm
{
    PositionalPid::PositionalPid(float kp, float ki, float kd, float out_min,
        float out_max, float integral_min, float integral_max) noexcept
        : kp_{kp},
          ki_{ki},
          kd_{kd},
          out_min_{out_min},
          out_max_{out_max},
          integral_min_{integral_min},
          integral_max_{integral_max}
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
        : kp_{kp},
          ki_{ki},
          kd_{kd},
          delta_out_min_{delta_out_min},
          delta_out_max_{delta_out_max}
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
