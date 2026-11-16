/**
 * @file voltage_limit.cpp
 * @brief αβ电压矢量限幅模块实现：六边形I区限幅、圆形内切圆限幅
 * @details
 * 纯算法实现，无硬件依赖；输出限幅后的αβ电压供给SVPWM使用。
 * @author Robot_FOC Project
 */
#include "ALGORITHM/voltage_limit.hpp"

#include <cmath>

namespace
{
    constexpr float SQRT3 {1.73205080757F};
    constexpr float SQRT3_HALF {0.8660254F};
    constexpr float HALF {0.5F};
}

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode VoltageLimit::hexagon_limit(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        VoltageLimitOut& result_out
    ) noexcept
    {
        const common::AlphaBetaVoltage& voltage_in = alphabeta_in;
        const common::Volts& bus = v_bus;
        VoltageLimitOut result {};
        float v_max {0.0F};
        float scale {1.0F};
        if ((!std::isfinite(voltage_in.alpha_volt_))
          ||(!std::isfinite(voltage_in.beta_volt_))
          ||(!std::isfinite(bus)
          ||(bus <= 0.0F)  
        )
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        v_max = bus / SQRT3;
        if ((!std::isfinite(v_max)) || (v_max <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        // 先按最大输入分量归一化，再计算六边形投影，避免有限大数相加时溢出。
        const float input_scale = std::fmaxf(
            std::abs(voltage_in.alpha_volt_),
            std::abs(voltage_in.beta_volt_)
        );
        if (input_scale > 0.0F)
        {
            const float alpha_normalized = voltage_in.alpha_volt_ / input_scale;
            const float beta_normalized = voltage_in.beta_volt_ / input_scale;
            const float projection_1 = std::abs(
                (SQRT3_HALF * alpha_normalized) + (HALF * beta_normalized)
            );
            const float projection_2 = std::abs(beta_normalized);
            const float projection_3 = std::abs(
                (-SQRT3_HALF * alpha_normalized) + (HALF * beta_normalized)
            );
            const float max_projection = std::fmaxf(
                std::fmaxf(projection_1, projection_2),
                projection_3
            );
            if ((!std::isfinite(max_projection)) || (max_projection <= 0.0F))
            {
                return common::ErrorCode::ErrorInvalidParam;
            }

            const float normalized_limit = v_max / input_scale;
            if (max_projection > normalized_limit)
            {
                scale = normalized_limit / max_projection;
                result.is_limited_ = true;
            }
        }
        if (!std::isfinite(scale))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        result.alphabeta_out_.alpha_volt_ = scale * voltage_in.alpha_volt_;
        result.alphabeta_out_.beta_volt_ = scale * voltage_in.beta_volt_;
        if (std::isfinite(result.alphabeta_out_.alpha_volt_) && std::isfinite(result.alphabeta_out_.beta_volt_))
        {
            result_out = result;
            return common::ErrorCode::Ok;
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode VoltageLimit::circle_limit(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        VoltageLimitOut& result_out
    ) noexcept
    {
        const common::AlphaBetaVoltage& voltage_in = alphabeta_in;
        const common::Volts& bus = v_bus;
        VoltageLimitOut result {};
        float v_max {0.0F};
        float amp {0.0F};
        if ((!std::isfinite(voltage_in.alpha_volt_))
          ||(!std::isfinite(voltage_in.beta_volt_))
          ||(!std::isfinite(bus)
          ||(bus <= 0.0F)  
        )
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        v_max = bus / SQRT3;
        amp = std::hypot(voltage_in.alpha_volt_, voltage_in.beta_volt_);
        if (!std::isfinite(amp)||!std::isfinite(v_max) || (v_max <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        float scale = 1.0F;
        if(amp > v_max)
        {
            scale = v_max / amp;
            result.is_limited_ = true;
        }
        result.alphabeta_out_.alpha_volt_ = scale * voltage_in.alpha_volt_;
        result.alphabeta_out_.beta_volt_  = scale * voltage_in.beta_volt_;
        if (std::isfinite(result.alphabeta_out_.alpha_volt_) && std::isfinite(result.alphabeta_out_.beta_volt_))
        {
            result_out = result;
            return common::ErrorCode::Ok;
        }
        return common::ErrorCode::ErrorInvalidParam;
    }
} // namespace robot_foc::algorithm
