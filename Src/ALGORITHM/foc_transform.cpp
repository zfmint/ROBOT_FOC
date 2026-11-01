/**
 * @file foc_transform.cpp
 * @brief FOC变换实现
 * @details
 *  - Clarke变换输入时三相电流/电压结构体，但可以只传输两相，由Clarke算法模块
 *      算出第三相
 *  - Clarke反变换直接输出三相电流/电压
 *  - Park变换/反变换需要有电角度计算
 * @note
 *  - Clarke变换公式
 *  三相电流
 *      Iα = 2/3 * （Ia - 1/2*Ib - 1/2*Ic）
 *      Iβ = 2/3 * （Ib - Ic） / √3
 *  两相电流
 *      Ia + Ib + Ic = 0
 *      Iα = 2/3 * Ia
 *      Iβ = 2/3 * (Ia + 2Ib) / √3
 *  -  Clarke逆变换公式
 *      Ia = Iα
 *      Ib = -1/2 * Ia + (√3/2) * Iβ
 *      Ic = -1/2 * Ia - (√3/2) * Iβ
 *  - Park变换公式
 *      Id = Iα*cosθ+Iβ*sinθ
 *      Iq = -Iα*sinθ+Iβ*cosθ
 *  - Park逆变换公式
 *      Iα = Id*cosθ-Iq*sinθ
 *      Iβ = Id*sinθ+Iq*cosθ
 */

#include "ALGORITHM/foc_transform.hpp"

#include <cmath>

namespace 
{
            
    constexpr float TWO_THREE {2.0F / 3.0F};
    constexpr float NEGHALF {-0.5F};
    constexpr float HALF {0.5F};
    constexpr float SQRT3 {1.73205080757F};

    /**
     * @brief 等幅值Clarke底层计算，三相输入版本
     * @param[in] a_input A相原始浮点值
     * @param[in] b_input B相原始浮点值
     * @param[in] c_input C相原始浮点值
     * @param[out] alpha_output α轴输出
     * @param[out] beta_output  β轴输出
     */    
    static void clarke_calculate_three(
        const float a_input,
        const float b_input,
        const float c_input,
        float& alpha_output,
        float& beta_output
    ) noexcept
    {
        alpha_output = TWO_THREE*(a_input - HALF*b_input - HALF*c_input); 
        beta_output = TWO_THREE*(b_input - c_input)/SQRT3;
    }

    /**
     * @brief 等幅值Clarke底层计算，两相AB输入版本
     * @details 内部隐含约束 C = -a_input - b_input
     * @param[in] a_input A相原始浮点值
     * @param[in] b_input B相原始浮点值
     * @param[out] alpha_output α轴输出
     * @param[out] beta_output  β轴输出
     */        
    static void clarke_calculate_two(
        const float a_input,
        const float b_input,
        float& alpha_output,
        float& beta_output
    ) noexcept
    {
        alpha_output = a_input;
        beta_output = TWO_THREE*(a_input+2*b_input)/SQRT3;
    }

    /**
     * @brief 逆Clarke底层计算 αβ → ABC三相
     * @param[in] alpha_input α轴输入
     * @param[in] beta_input  β轴输入
     * @param[out] a_output A相输出
     * @param[out] b_output B相输出
     * @param[out] c_output C相输出
     */        
    static void inverse_clarke_calculate(
        const float alpha_input,
        const float beta_input,
        float& a_output,
        float& b_output,
        float& c_output
    ) noexcept
    {
        a_output = alpha_input;
        b_output = NEGHALF * alpha_input + (SQRT3 / 2.0F) * beta_input;
        c_output = NEGHALF * alpha_input - (SQRT3 / 2.0F) * beta_input;
    }

    /**
     * @brief Park底层旋转变换 αβ → dq
     * @param[in] alpha_input α轴输入
     * @param[in] beta_input  β轴输入
     * @param[in] theta_e    电角度（rad）
     * @param[out] d_output d轴输出
     * @param[out] q_output q轴输出
     */        
    static void park_calculate(
        const float alpha_input,
        const float beta_input,
        const float theta_e,
        float& d_output,
        float& q_output
    ) noexcept
    {
        const float cos_ang = std::cos(theta_e);
        const float sin_ang = std::sin(theta_e);
        d_output = alpha_input * cos_ang + beta_input * sin_ang;
        q_output = -alpha_input * sin_ang + beta_input * cos_ang;
    }

    /**
     * @brief 逆Park底层旋转变换 dq → αβ
     * @param[in] d_input d轴输入
     * @param[in] q_input q轴输入
     * @param[in] theta_e 电角度（rad）
     * @param[out] alpha_output α轴输出
     * @param[out] beta_output  β轴输出
     */
    static void inverse_park_calculate(
        const float d_input,
        const float q_input,
        const float theta_e,
        float& alpha_output,
        float& beta_output
    ) noexcept
    {
        const float cos_ang = std::cos(theta_e);
        const float sin_ang = std::sin(theta_e);
        alpha_output = d_input * cos_ang - q_input * sin_ang;
        beta_output  = d_input * sin_ang + q_input * cos_ang;
    }
} // namespace 

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode FocTransform::clarke(const ABCCurrent& abc_current_in, AlphaBetaCurrent& ab_current_out) noexcept
    {
        common::Amperes a_curr = abc_current_in.a_current_;
        common::Amperes b_curr = abc_current_in.b_current_;
        common::Amperes c_curr = abc_current_in.c_current_;
        common::Amperes alpha_curr_out {0.0F};
        common::Amperes beta_curr_out {0.0F};
        if (std::isfinite(a_curr)&&std::isfinite(b_curr)&&std::isfinite(c_curr))
        {
            clarke_calculate_three(a_curr,b_curr,c_curr,alpha_curr_out,beta_curr_out);
            if (std::isfinite(alpha_curr_out)&&std::isfinite(beta_curr_out))
            {
                ab_current_out.alpha_current_ = alpha_curr_out;
                ab_current_out.beta_current_ = beta_curr_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::clarke(const ABCurrent& ab_current_in, AlphaBetaCurrent& ab_current_out) noexcept
    {
        common::Amperes a_curr = ab_current_in.a_current_;
        common::Amperes b_curr = ab_current_in.b_current_;
        common::Amperes alpha_curr_out {0.0F};
        common::Amperes beta_curr_out {0.0F};
        if (std::isfinite(a_curr)&&std::isfinite(b_curr))
        {
            clarke_calculate_two(a_curr,b_curr,alpha_curr_out,beta_curr_out);
            if (std::isfinite(alpha_curr_out)&&std::isfinite(beta_curr_out))
            {
                ab_current_out.alpha_current_ = alpha_curr_out;
                ab_current_out.beta_current_ = beta_curr_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::clarke(const ABCVoltage& abc_voltage_in, AlphaBetaVoltage& ab_voltage_out) noexcept
    {
        common::Volts a_volt = abc_voltage_in.a_volt_;
        common::Volts b_volt = abc_voltage_in.b_volt_;
        common::Volts c_volt = abc_voltage_in.c_volt_;
        common::Volts alpha_volt_out {0.0F};
        common::Volts beta_volt_out {0.0F};
        if (std::isfinite(a_volt)&&std::isfinite(b_volt)&&std::isfinite(c_volt))
        {
            clarke_calculate_three(a_volt,b_volt,c_volt,alpha_volt_out,beta_volt_out);
            if (std::isfinite(alpha_volt_out)&&std::isfinite(beta_volt_out))
            {
                ab_voltage_out.alpha_volt_ = alpha_volt_out;
                ab_voltage_out.beta_volt_ = beta_volt_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }
    
    [[nodiscard]] common::ErrorCode FocTransform::clarke(const ABVoltage& ab_voltage_in, AlphaBetaVoltage& ab_voltage_out) noexcept
    {
        common::Volts a_volt = ab_voltage_in.a_volt_;
        common::Volts b_volt = ab_voltage_in.b_volt_;
        common::Volts alpha_volt_out {0.0F};
        common::Volts beta_volt_out {0.0F};
        if (std::isfinite(a_volt)&&std::isfinite(b_volt))
        {
            clarke_calculate_two(a_volt,b_volt,alpha_volt_out,beta_volt_out);
            if (std::isfinite(alpha_volt_out)&&std::isfinite(beta_volt_out))
            {
                ab_voltage_out.alpha_volt_ = alpha_volt_out;
                ab_voltage_out.beta_volt_ = beta_volt_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::inverse_clarke(const AlphaBetaCurrent& alphabeta_current_in, ABCCurrent& abc_current_out) noexcept
    {
        common::Amperes alpha_curr_in = alphabeta_current_in.alpha_current_;
        common::Amperes beta_curr_in = alphabeta_current_in.beta_current_;
        common::Amperes a_curr_out {0.0F};
        common::Amperes b_curr_out {0.0F};
        common::Amperes c_curr_out {0.0F};
        if (std::isfinite(alpha_curr_in)&&std::isfinite(beta_curr_in))
        {
            inverse_clarke_calculate(alpha_curr_in,beta_curr_in,a_curr_out,b_curr_out,c_curr_out);
            if (std::isfinite(a_curr_out)&&std::isfinite(b_curr_out)&&std::isfinite(c_curr_out))
            {
                abc_current_out.a_current_ = a_curr_out;
                abc_current_out.b_current_ = b_curr_out;
                abc_current_out.c_current_ = c_curr_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::inverse_clarke(const AlphaBetaVoltage& alphabeta_voltage_in, ABCVoltage& abc_voltage_out) noexcept
    {
        common::Volts alpha_volt_in = alphabeta_voltage_in.alpha_volt_;
        common::Volts beta_volt_in = alphabeta_voltage_in.beta_volt_;
        common::Volts a_volt_out {0.0F};
        common::Volts b_volt_out {0.0F};
        common::Volts c_volt_out {0.0F};
        if (std::isfinite(alpha_volt_in)&&std::isfinite(beta_volt_in))
        {
            inverse_clarke_calculate(alpha_volt_in,beta_volt_in,a_volt_out,b_volt_out,c_volt_out);
            if (std::isfinite(a_volt_out)&&std::isfinite(b_volt_out)&&std::isfinite(c_volt_out))
            {
                abc_voltage_out.a_volt_ = a_volt_out;
                abc_voltage_out.b_volt_ = b_volt_out;
                abc_voltage_out.c_volt_ = c_volt_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::park(const AlphaBetaCurrent& alphabeta_current_in, const common::ElectricalRad theta_e, DQCurrent& dq_current_out) noexcept
    {
        common::Amperes alpha_curr_in = alphabeta_current_in.alpha_current_;
        common::Amperes beta_curr_in = alphabeta_current_in.beta_current_;
        common::ElectricalRad ang = theta_e;
        common::Amperes d_curr_out {0.0F};
        common::Amperes q_curr_out {0.0F};
        if (std::isfinite(alpha_curr_in)&&std::isfinite(beta_curr_in)&&std::isfinite(ang))
        {
            park_calculate(alpha_curr_in,beta_curr_in,ang,d_curr_out,q_curr_out);
            if (std::isfinite(d_curr_out)&&std::isfinite(q_curr_out))
            {
                dq_current_out.d_current_ = d_curr_out;
                dq_current_out.q_current_ = q_curr_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::park(const AlphaBetaVoltage& alphabeta_voltage_in, const common::ElectricalRad theta_e, DQVoltage& dq_voltage_out) noexcept
    {
        common::Volts alpha_volt_in = alphabeta_voltage_in.alpha_volt_;
        common::Volts beta_volt_in = alphabeta_voltage_in.beta_volt_;
        common::ElectricalRad ang = theta_e;
        common::Volts d_volt_out {0.0F};
        common::Volts q_volt_out {0.0F};
        if (std::isfinite(alpha_volt_in)&&std::isfinite(beta_volt_in)&&std::isfinite(ang))
        {
            park_calculate(alpha_volt_in,beta_volt_in,ang,d_volt_out,q_volt_out);
            if (std::isfinite(d_volt_out)&&std::isfinite(q_volt_out))
            {
                dq_voltage_out.d_volt_ = d_volt_out;
                dq_voltage_out.q_volt_ = q_volt_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::inverse_park(const DQCurrent& dq_current_in, const common::ElectricalRad theta_e, AlphaBetaCurrent& alphabeta_current_out) noexcept
    {
        common::Amperes d_curr_in = dq_current_in.d_current_;
        common::Amperes q_curr_in = dq_current_in.q_current_;
        common::ElectricalRad ang = theta_e;
        common::Amperes alpha_curr_out {0.0F};
        common::Amperes beta_curr_out {0.0F};
        if (std::isfinite(d_curr_in)&&std::isfinite(q_curr_in)&&std::isfinite(ang))
        {
            inverse_park_calculate(d_curr_in,q_curr_in,ang,alpha_curr_out,beta_curr_out);
            if (std::isfinite(alpha_curr_out)&&std::isfinite(beta_curr_out))
            {
                alphabeta_current_out.alpha_current_ = alpha_curr_out;
                alphabeta_current_out.beta_current_ = beta_curr_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }

    [[nodiscard]] common::ErrorCode FocTransform::inverse_park(const DQVoltage& dq_voltage_in, const common::ElectricalRad theta_e, AlphaBetaVoltage& alphabeta_voltage_out) noexcept
    {
        common::Volts d_volt_in = dq_voltage_in.d_volt_;
        common::Volts q_volt_in = dq_voltage_in.q_volt_;
        common::ElectricalRad ang = theta_e;
        common::Volts alpha_volt_out {0.0F};
        common::Volts beta_volt_out {0.0F};
        if (std::isfinite(d_volt_in)&&std::isfinite(q_volt_in)&&std::isfinite(ang))
        {
            inverse_park_calculate(d_volt_in,q_volt_in,ang,alpha_volt_out,beta_volt_out);
            if (std::isfinite(alpha_volt_out)&&std::isfinite(beta_volt_out))
            {
                alphabeta_voltage_out.alpha_volt_ = alpha_volt_out;
                alphabeta_voltage_out.beta_volt_ = beta_volt_out;
                return common::ErrorCode::Ok;
            }
        }
        return common::ErrorCode::ErrorInvalidParam;
    }
} // namespace robot_foc::algorithm
