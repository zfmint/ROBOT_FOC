/**
 * @file voltage_limit.cpp
 * @brief αβ电压矢量限幅模块实现：六边形I区限幅、圆形内切圆限幅
 * @details
 * 纯算法实现，无硬件依赖；输出限幅后的αβ电压供给SVPWM使用。
 * @author Robot_FOC Project
 */
#include "ALGORITHM/voltage_limit.hpp"

#include <cmath>
#include <cstdint>

namespace
{
    constexpr float SQRT3 {1.73205080757F};
}

namespace robot_foc::algorithm
{

    [[nodiscard]] common::ErrorCode VoltageLimit::hexagon_limit(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        common::AlphaBetaVoltage& alphabeta_out
    ) noexcept
    {
        const common::AlphaBetaVoltage& voltage_in = alphabeta_in;
        common::Volts alpha_out{0.0F};
        common::Volts beta_out{0.0F};

        // 优先校验合法性，零矢量也必须校验Vbus
        if (!std::isfinite(voltage_in.alpha_volt_)
            || !std::isfinite(voltage_in.beta_volt_)
            || !std::isfinite(v_bus)
            || (v_bus <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        // 零矢量直接返回
        if ((voltage_in.alpha_volt_ == 0.0F) && (voltage_in.beta_volt_ == 0.0F))
        {
            alphabeta_out.alpha_volt_ = alpha_out;
            alphabeta_out.beta_volt_ = beta_out;
            return common::ErrorCode::Ok;
        }

        // 限幅模块独立完成扇区识别，不依赖Svpwm的私有实现。
        std::uint8_t sector{0U};
        const float alpha_f = static_cast<float>(voltage_in.alpha_volt_);
        const float beta_f  = static_cast<float>(voltage_in.beta_volt_);
        const float vbus_f = static_cast<float>(v_bus);

        const float x = beta_f;
        const float y = SQRT3 * alpha_f - beta_f;
        const float z = -SQRT3 * alpha_f - beta_f;
        if ((x >= 0)&&(y > 0)&&(z < 0))          sector = 1U;
        else if ((x > 0)&&(y <= 0)&&(z < 0))    sector = 2U;
        else if ((x > 0)&&(y < 0)&&(z >= 0))    sector = 3U;
        else if ((x <= 0)&&(y < 0)&&(z > 0))    sector = 4U;
        else if ((x < 0)&&(y >= 0)&&(z > 0))    sector = 5U;
        else if ((x < 0)&&(y > 0)&&(z <= 0))    sector = 6U;
        else
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        // 扇区旋转折叠到sector1
        constexpr float kCosTable[] = {1.0F, 0.5F, -0.5F, -1.0F, -0.5F, 0.5F};
        constexpr float kSinTable[] = {0.0F, -0.8660254F, -0.8660254F, 0.0F, 0.8660254F, 0.8660254F};
        const std::uint8_t idx = sector - 1U;
        const float alpha_rot = alpha_f * kCosTable[idx] - beta_f * kSinTable[idx];
        const float beta_rot  = alpha_f * kSinTable[idx] + beta_f * kCosTable[idx];

        const float t1 = (3.0F * alpha_rot - SQRT3 * beta_rot) / (2.0F * vbus_f);
        const float t2 = (SQRT3 * beta_rot) / vbus_f;
        if(!std::isfinite(t1) || !std::isfinite(t2))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        const float sum = t1 + t2;
        float k = 1.0F;
        if(sum > 1.0F)
        {
            k = 1.0F / sum;
        }
        alpha_out = static_cast<common::Volts>(k * alpha_f);
        beta_out  = static_cast<common::Volts>(k * beta_f);

        alphabeta_out.alpha_volt_ = alpha_out;
        alphabeta_out.beta_volt_  = beta_out;
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode VoltageLimit::circle_limit(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        common::AlphaBetaVoltage& alphabeta_out
    ) noexcept
    {
        const common::AlphaBetaVoltage& voltage_in = alphabeta_in;
        if (!std::isfinite(voltage_in.alpha_volt_)
            || !std::isfinite(voltage_in.beta_volt_)
            || !std::isfinite(v_bus)
            || (v_bus <= 0.0F))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }

        const float alpha_f = static_cast<float>(voltage_in.alpha_volt_);
        const float beta_f  = static_cast<float>(voltage_in.beta_volt_);
        const float vbus_f = static_cast<float>(v_bus);
        const float v_max = vbus_f / SQRT3;

        // hypot避免直接平方相加造成不必要的中间结果溢出。
        const float amp = std::hypot(alpha_f, beta_f);
        if (!std::isfinite(amp))
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        float scale = 1.0F;
        if(amp > 1e-9F)
        {
            scale = v_max / amp;
        }
        if(scale > 1.0F)
        {
            scale = 1.0F;
        }

        alphabeta_out.alpha_volt_ = static_cast<common::Volts>(scale * alpha_f);
        alphabeta_out.beta_volt_  = static_cast<common::Volts>(scale * beta_f);
        return common::ErrorCode::Ok;
    }

} // namespace robot_foc::algorithm
