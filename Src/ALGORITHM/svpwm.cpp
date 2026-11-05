/**
 * @file svpwm.cpp
 * @brief 七段式SVPWM空间矢量调制算法实现，等幅值体系
 * @details
 * 实现七段对称SVPWM，零矢量居中；输入静止坐标系αβ电压矢量
 * 输出三相PWM占空比，输出范围限定 0.0F ~ 1.0F。
 * 本模块无运行时历史状态，函数可重入。
 *
 * @note
 *  1. 依赖母线电压Vbus进行归一化计算；
 *  2. 辅助计算函数放置于匿名命名空间内，对外不可见；
 *  3. 与foc_transform保持一致，全程使用等幅值变换体系。
 *  4. 扇区换算公式
 *      Uα' = Uα * cosθ - Uβ * sinθ
 *      Uβ' = Uα * sinθ + Uβ * cosθ
 *  5. 扇区判断
 * X = Uβ
 * Y = √3Uα - Uβ
 * Z = -√3Uα - Uβ
 *      X	Y	Z
 * I	+	+	-
 * II	+	-	-
 * III	+	-	+
 * IV	-	-	+
 * V	-	+	+
 * VI	-	+	-
 *  6. 扇区边界采用左闭右开区间
 * 边界角度	为零的判断量	归属扇区
 *  0°	        X=0	        I
 *  60°     	Y=0	        II
 *  120°    	Z=0	        III
 *  180°    	X=0	        IV
 *  240°    	Y=0     	V
 *  300°       	Z=0     	VI
 * @author Robot_FOC Project
 */
#include "ALGORITHM/svpwm.hpp"

#include <cmath>
#include <cstdint>

#include "units.hpp"

namespace
{
    constexpr float HALF {0.5F};
    constexpr float SQRT3 {1.73205080757F};
    constexpr float SQRT3_HALF {0.8660254F};
    constexpr float Tolerance{1.0e-4F};
    constexpr float cos_table [] = {
        1.000000F,
        0.500000F,
        -0.500000F,
        -1.000000F,
        -0.500000F,
        0.500000F
    };
    constexpr float sin_table [] = {
        0.000000F,
        -SQRT3_HALF,
        -SQRT3_HALF,
        0.000000F,
        SQRT3_HALF,
        SQRT3_HALF
    };
    enum
    {
        Number1 = 1,
        Number2 = 2,
        Number3 = 3,
        Number4 = 4,
        Number5 = 5,
        Number6 = 6
    };
} // namespace


namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode Svpwm::svpwm_detect_sector(
        const common::AlphaBetaVoltage& alphabeta_in,
        std::uint8_t& sector_out
    ) noexcept
    {
        common::Volts alpha_in = alphabeta_in.alpha_volt_;
        common::Volts beta_in = alphabeta_in.beta_volt_;
        std::uint8_t sector {0};
        float x = beta_in;
        float y = SQRT3*alpha_in - beta_in;
        float z = -SQRT3*alpha_in - beta_in;
        if ((x >= 0)&&(y > 0)&&(z < 0))
        {
            sector = 1U;
        }
        else if ((x > 0)&&(y <= 0)&&(z < 0))
        {
            sector = 2U;
        }
        else if ((x > 0)&&(y < 0)&&(z >= 0))
        {
            sector = 3U;
        }
        else if ((x <= 0)&&(y < 0)&&(z > 0))
        {
            sector = 4U;
        }            
        else if ((x < 0)&&(y >= 0)&&(z > 0))
        {
            sector = 5U;
        }
        else if ((x < 0)&&(y > 0)&&(z <= 0))
        {
            sector = 6U;
        }
        else
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        sector_out = sector;
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] static common::ErrorCode svpwm_to_one_sector(
        const common::AlphaBetaVoltage& alphabeta_in,
        const std::uint8_t sector_in,
        common::AlphaBetaVoltage& alphabeta_out
    ) noexcept
    {
        common::Volts alpha_in = alphabeta_in.alpha_volt_;
        common::Volts beta_in = alphabeta_in.beta_volt_;
        common::Volts alpha_out {0.0F};
        common::Volts beta_out {0.0F};
        std::uint8_t sector = sector_in - 1;
        if(sector > 5)
        {
            return common::ErrorCode::ErrorInvalidParam;
        }
        alpha_out = alpha_in*cos_table[sector]-beta_in*sin_table[sector];
        beta_out = alpha_in*sin_table[sector]+beta_in*cos_table[sector];
        if (std::isfinite(alpha_out) && std::isfinite(beta_out))
        {
            alphabeta_out.alpha_volt_ = alpha_out;
            alphabeta_out.beta_volt_ = beta_out;
            return common::ErrorCode::Ok;
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] static common::ErrorCode svpwm_calculate_time(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        SvpwmTime& time_out
    ) noexcept
    {
        common::Volts alpha_in = alphabeta_in.alpha_volt_;
        common::Volts beta_in = alphabeta_in.beta_volt_;
        common::Volts vtotal = v_bus;
        float t1 {0.0F};
        float t2 {0.0F};
        float t0 {0.0F};
        t1 = (3.0F*alpha_in - SQRT3*beta_in)/vtotal*HALF;
        t2 = (SQRT3*beta_in)/vtotal;
        t0 = 1.0F-t1-t2;
        if ((t0 >= -Tolerance) && (t1 >= -Tolerance) && (t2 >= -Tolerance))
        {
            if (t0 < 0)
            {
                t1 = t1 + t0*HALF;
                t2 = t2 + t0*HALF;
                t0 = 0.0F;
            }
            if ((t1 < 0) || (t2 < 0))
            {
                return common::ErrorCode::ErrorInvalidParam;
            }
            if (std::isfinite(t0)&&std::isfinite(t1)&&std::isfinite(t2))
            {
                time_out.t1_ratio_ = t1;
                time_out.t2_ratio_ = t2;
                time_out.t0_ratio_ = t0;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] static common::ErrorCode svpwm_calculate_duty(
        const std::uint8_t sector_in,
        const SvpwmTime time_in,
        SvpwmDuty& duty_out
    ) noexcept;

    [[nodiscard]] common::ErrorCode Svpwm::voltage_limit(
        const common::AlphaBetaVoltage& alphabeta_in,
        const common::Volts v_bus,
        common::AlphaBetaVoltage& alphabeta_out
    ) const noexcept
    {

    }

    [[nodiscard]] common::ErrorCode Svpwm::svpwm_calculate(
        const common::AlphaBetaVoltage& alphabeta_in,
        common::Volts v_bus,
        SvpwmDuty& duty_out
    ) const noexcept
    {

    }
} // namespace robot_foc::algorithm
