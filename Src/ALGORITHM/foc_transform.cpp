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
 *      Ia + Ib + Ic = 0
 *      Iα = 2/3 * Ia
 *      Iβ = 2/3 * (Ia + 2Ib)/√3
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

namespace robot_foc::algorithm
{

    constexpr float two_three = 2.0F / 3.0F;
    constexpr float neghalf = -0.5F;

    [[nodiscard]] common::ErrorCode clarke_current(const ABCCurrent& abc_current_in, AlphaBetaCurrent& ab_current_out) noexcept
    {
        ab_current_out.alpha_current_ = two_three*abc_current_in.a_current_;
        ab_current_out.beta_current_ = two_three*(abc_current_in.a_current_+2*abc_current_in.b_current_)/sqrt(3);
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode inverse_clarke_current(const AlphaBetaCurrent& ab_current_in, ABCCurrent& abc_current_out) noexcept
    {
        abc_current_out.a_current_ = ab_current_in.alpha_current_;
        abc_current_out.b_current_ = neghalf * ab_current_in.alpha_current_ + (std::sqrt(3) / 2.0F) * ab_current_in.beta_current_;
        abc_current_out.c_current_ = neghalf * ab_current_in.alpha_current_ - (std::sqrt(3) / 2.0F) * ab_current_in.beta_current_;
        return common::ErrorCode::Ok;
    }
    
    /**
     * @brief 等幅值Clarke变换（电压版本）
     * @param[in] abc_volt_in 三相电压输入
     * @param[out] ab_volt_out 两相静止坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode clarke_volt(const ABCVolts& abc_volt_in, AlphaBetaVolts& ab_volt_out) noexcept
    {
        ab_volt_out.alpha_volt_ = two_three*abc_volt_in.a_volt_;
        ab_volt_out.beta_volt_ = two_three*(abc_volt_in.a_volt_+2*abc_volt_in.b_volt_)/sqrt(3);
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode inverse_clarke_volt(const AlphaBetaVolts& ab_volt_in, ABCVolts& abc_volt_out) noexcept
    {
        abc_volt_out.a_volt_ = ab_volt_in.alpha_volt_;
        abc_volt_out.b_volt_ = neghalf * ab_volt_in.alpha_volt_ + (std::sqrt(3) / 2.0F) * ab_volt_in.beta_volt_;
        abc_volt_out.c_volt_ = neghalf * ab_volt_in.alpha_volt_ - (std::sqrt(3) / 2.0F) * ab_volt_in.beta_volt_;
        return common::ErrorCode::Ok;
    }
    
    [[nodiscard]] common::ErrorCode park_current(const AlphaBetaCurrent& ab_current_in, common::ElectricalRad theta_e, DQCurrent& dq_current_out) noexcept
    {
        dq_current_out.d_current_ = ab_current_in.alpha_current_*(std::cos(theta_e))+ab_current_in.beta_current_*(std::sin(theta_e));
        dq_current_out.q_current_ = -ab_current_in.alpha_current_*(std::sin(theta_e))+ab_current_in.beta_current_*(std::cos(theta_e));
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode inverse_park_current(const DQCurrent& dq_current_in, common::ElectricalRad theta_e, AlphaBetaCurrent& ab_current_out) noexcept
    {
        ab_current_out.alpha_current_ = dq_current_in.d_current_*(std::cos(theta_e))-dq_current_in.q_current_*(std::sin(theta_e));
        ab_current_out.beta_current_ = dq_current_in.d_current_*(std::sin(theta_e))+dq_current_in.q_current_*(std::cos(theta_e));
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode park_volt(const AlphaBetaVolts& ab_volt_in, common::ElectricalRad theta_e, DQVolts& dq_volt_out) noexcept
    {
        dq_volt_out.d_volt_ = ab_volt_in.alpha_volt_*(std::cos(theta_e))+ab_volt_in.beta_volt_*(std::sin(theta_e));
        dq_volt_out.q_volt_ = -ab_volt_in.alpha_volt_*(std::sin(theta_e))+ab_volt_in.beta_volt_*(std::cos(theta_e));
        return common::ErrorCode::Ok;
    }

    [[nodiscard]] common::ErrorCode inverse_park_volt(const DQVolts& dq_volt_in, common::ElectricalRad theta_e, AlphaBetaVolts& ab_volt_out) noexcept
    {
        ab_volt_out.alpha_volt_ = dq_volt_in.d_volt_*(std::cos(theta_e))-dq_volt_in.q_volt_*(std::sin(theta_e));
        ab_volt_out.beta_volt_ = dq_volt_in.d_volt_*(std::sin(theta_e))+dq_volt_in.q_volt_*(std::cos(theta_e));
        return common::ErrorCode::Ok;
    }

} // namespace robot_foc::algorithm




