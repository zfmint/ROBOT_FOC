/**
 * @file foc_transform.cpp
 * @brief FOC坐标变换模块实现
 *
 * @details
 *  本模块实现等幅值约定下的 Clarke / 逆Clarke、Park / 逆Park 坐标变换，
 *  用于永磁同步电机FOC控制链路，电压、电流变换共用同一套变换公式。
 *
 *  - Clarke变换：支持两种输入模式
 *    1. 三相输入模式：传入独立采样 Ia、Ib、Ic，不强制满足 Ia+Ib+Ic=0，可处理零序、不平衡分量
 *    2. 两相输入模式：仅传入 Ia、Ib，数学约束 Ia+Ib+Ic=0，内部推导 Ic = -Ia - Ib，适用于双电阻采样方案
 *  - 逆Clarke变换：αβ域矢量还原为三相量，输出 Ia、Ib、Ic
 *  - Park变换：静止αβ坐标系 → 旋转dq坐标系，输入电角度 θ_e
 *  - 逆Park变换：旋转dq坐标系 → 静止αβ坐标系，输入电角度 θ_e
 *
 * @note
 *  【变换约定：等幅值 Clarke】
 *  变换前后相电压/相电流峰值保持相等；整套变换系数配对，禁止与等功率Clarke混用。
 *
 *  Clarke 三相输入公式：
 *  Iα = 2/3 * (Ia - 1/2*Ib - 1/2*Ic)
 *  Iβ = 2/3 * (Ib - Ic) / √3
 *
 *  Clarke 两相输入（约束 Ia+Ib+Ic=0）：
 *  Iα = 2/3 * Ia
 *  Iβ = 2/3 * (Ia + 2*Ib) / √3
 *
 *  逆Clarke（αβ → abc，等幅值配套）：
 *  Ia = Iα
 *  Ib = -1/2 * Iα + (√3/2) * Iβ
 *  Ic = -1/2 * Iα - (√3/2) * Iβ
 *
 *  Park 变换（αβ → dq）：
 *  Id = Iα*cosθ_e + Iβ*sinθ_e
 *  Iq = -Iα*sinθ_e + Iβ*cosθ_e
 *
 *  逆Park 变换（dq → αβ）：
 *  Iα = Id*cosθ_e - Iq*sinθ_e
 *  Iβ = Id*sinθ_e + Iq*cosθ_e
 *
 *  @attention
 *  1. 两相Clarke仅在硬件为双电阻采样、三相和为0的前提下使用；存在零序/不平衡时必须使用三相输入版本
 *  2. θ_e 为电角度，单位 rad，角度定义：d轴相对α轴的夹角
 *  3. 本套为等幅值体系，若切换等功率Clarke，正向、逆Clarke系数必须同步替换，不可交叉混用
 *  4. Park、逆Park为纯旋转变换，矢量模长不缩放，等幅值/等功率两套体系可共用这组公式
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
        alpha_output = TWO_THREE * (a_input - HALF * b_input - HALF * c_input);
        beta_output  = TWO_THREE * ( (SQRT3 / 2.0F) * b_input - (SQRT3 / 2.0F) * c_input );
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
        beta_output = (a_input + 2.0F * b_input) / SQRT3;
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
    [[nodiscard]] common::ErrorCode FocTransform::clarke(const common::ABCCurrent& abc_current_in, common::AlphaBetaCurrent& ab_current_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::clarke(const common::ABCurrent& ab_current_in, common::AlphaBetaCurrent& ab_current_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::clarke(const common::ABCVoltage& abc_voltage_in, common::AlphaBetaVoltage& ab_voltage_out) noexcept
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
    
    [[nodiscard]] common::ErrorCode FocTransform::clarke(const common::ABVoltage& ab_voltage_in, common::AlphaBetaVoltage& ab_voltage_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::inverse_clarke(const common::AlphaBetaCurrent& alphabeta_current_in, common::ABCCurrent& abc_current_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::inverse_clarke(const common::AlphaBetaVoltage& alphabeta_voltage_in, common::ABCVoltage& abc_voltage_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::park(const common::AlphaBetaCurrent& alphabeta_current_in, const common::ElectricalRad theta_e, common::DQCurrent& dq_current_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::park(const common::AlphaBetaVoltage& alphabeta_voltage_in, const common::ElectricalRad theta_e, common::DQVoltage& dq_voltage_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::inverse_park(const common::DQCurrent& dq_current_in, const common::ElectricalRad theta_e, common::AlphaBetaCurrent& alphabeta_current_out) noexcept
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

    [[nodiscard]] common::ErrorCode FocTransform::inverse_park(const common::DQVoltage& dq_voltage_in, const common::ElectricalRad theta_e, common::AlphaBetaVoltage& alphabeta_voltage_out) noexcept
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
