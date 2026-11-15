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
        float max_projection {0.0F}, projection_1 {0.0F}, projection_2 {0.0F}, projection_3 {0.0F};
        float scale {1.0F};
        if ((!std::isfinite(voltage_in.alpha_volt))
          ||(!std::isfinite(voltage_in.beta_volt))
          ||(!std::isfinite(bus)
          ||(bus <= 0.0F)  
        )
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        v_max = bus / SQRT3;
        projection_1 = std::abs(SQRT3_HALF*voltage_in.alpha_volt + HALF*voltage_in.beta_volt);
        projection_2 = std::abs(voltage_in.beta_volt);
        projection_3 = std::abs(-SQRT3_HALF*voltage_in.alpha_volt + HALF*voltage_in.beta_volt);
        max_projection = std::fmaxf(std::fmaxf(projection_1,projection_2),projection_3);
        if (max_projection > v_max)
        {
            scale = v_max / max_projection;
            result.is_limited = true;
        }
        result.alphabeta_out.alpha_volt = scale*voltage_in.alpha_volt;
        result.alphabeta_out.beta_volt = scale*voltage_in.beta_volt;
        if (std::isfinite(result.alphabeta_out.alpha_volt) && std::isfinite(result.alphabeta_out.beta_volt))
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
        float k {1.0F};
        float amp {0.0F};
        if ((!std::isfinite(voltage_in.alpha_volt))
          ||(!std::isfinite(voltage_in.beta_volt))
          ||(!std::isfinite(bus)
          ||(bus <= 0.0F)  
        )
        )
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        v_max = bus / SQRT3;
        amp = std::hypot(voltage_in.alpha_volt, voltage_in.beta_volt);
        if (!std::isfinite(amp)||!std::isfinite(v_max) || (v_max <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        float scale = 1.0F;
        if(amp > v_max)
        {
            scale = v_max / amp;
            result.is_limited = true;
        }
        result.alphabeta_out.alpha_volt = scale * voltage_in.alpha_volt;
        result.alphabeta_out.beta_volt  = scale * voltage_in.beta_volt;
        if (std::isfinite(result.alphabeta_out.alpha_volt) && std::isfinite(result.alphabeta_out.beta_volt))
        {
            result_out = result;
            return common::ErrorCode::Ok;
        }
        return common::ErrorCode::ErrorInvalidParam;
    }
} // namespace robot_foc::algorithm
