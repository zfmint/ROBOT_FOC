/**
 * @file foc_transform.hpp
 * @brief FOC Clarke / Park / InverseClarke / InversePark坐标变换
 * @details
 * 采用【等幅值 Clarke】；所有角度统一为电角度，单位 rad；
 * 电流:A，电压:V；全部为无状态纯数学变换，无内部缓存。
 *
 * 变换清单：
 *  1. Clarke: 三相Ia,Ib,Ic → Iα,Iβ 静止两相正交
 *  2. Park:   Iα,Iβ → Id,Iq  同步旋转dq
 *  3. InversePark: Vd,Vq → Vα,Vβ
 *
 * @author Robot_FOC Project
 * @note
 *  - 角度必须是【电角度】，不是机械角度
 *  - 角度范围不限，内部直接送入sin/cos，由三角函数处理周期
 *  - Clarke 默认等幅值，不是等功率
 */
#ifndef ROBOT_FOC_ALGORITHM_FOC_TRANSFORM_HPP
#define ROBOT_FOC_ALGORITHM_FOC_TRANSFORM_HPP

#include <cmath>
#include "COMMON/units.hpp"
#include "COMMON/error_code.hpp"

namespace robot_foc::algorithm
{
    struct ABCCurrent
    {
        common::Amperes a_current_ = 0.0F;
        common::Amperes b_current_ = 0.0F;
        common::Amperes c_current_ = 0.0F;
    };

    struct AlphaBetaCurrent
    {
        common::Amperes alpha_current_ = 0.0F;
        common::Amperes beta_current_ = 0.0F;
    };

    struct DQCurrent
    {
        common::Amperes d_current_ = 0.0F;
        common::Amperes q_current_ = 0.0F;
    };
    
    struct ABCVolts
    {
        common::Volts a_volt_ = 0.0F;
        common::Volts b_volt_ = 0.0F;
        common::Volts c_volt_ = 0.0F;
    };

    struct AlphaBetaVolts
    {
        common::Volts alpha_volt_ = 0.0F;
        common::Volts beta_volt_ = 0.0F;
    };

    struct DQVolts
    {
        common::Volts d_volt_ = 0.0F;
        common::Volts q_volt_ = 0.0F;
    };

    /**
     * @brief 等幅值Clarke变换（电流版本）
     * @param[in] abc_current_in 三相电流输入
     * @param[out] ab_current_out 两相静止坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode clarke_current(const ABCCurrent& abc_current_in, AlphaBetaCurrent& ab_current_out) noexcept;

    /**
     * @brief 等幅值反Clarke变换（电流版本）
     * @param[in] ab_current_in 两相静止坐标系输入
     * @param[out] abc_current_out 三相电流输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode inverse_clarke_current(const AlphaBetaCurrent& ab_current_in, ABCCurrent& abc_current_out) noexcept;
    
    /**
     * @brief 等幅值Clarke变换（电压版本）
     * @param[in] abc_volt_in 三相电压输入
     * @param[out] ab_volt_out 两相静止坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode clarke_volt(const ABCVolts& abc_volt_in, AlphaBetaVolts& ab_volt_out) noexcept;

    /**
     * @brief 等幅值反Clarke变换（电压版本）
     * @param[in] ab_volt_in 两相静止坐标系输入
     * @param[out] abc_volt_out 三相电压输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode inverse_clarke_volt(const AlphaBetaVolts& ab_volt_in, ABCVolts& abc_volt_out) noexcept;
    
    /**
     * @brief 等幅值Park变换（电流版本）
     * @param[in] ab_current_in 两相静止坐标系输入
     * @param[out] dq_current_out 两相旋转坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode park_current(const AlphaBetaCurrent& ab_current_in, DQCurrent& dq_current_out) noexcept;

    /**
     * @brief 等幅值反Park变换（电流版本）
     * @param[in] dq_current_in 两相旋转坐标系输入
     * @param[out] ab_current_out 两相静止坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode inverse_park_current(const DQCurrent& dq_current_in, AlphaBetaCurrent& ab_current_out) noexcept;

    /**
     * @brief 等幅值Park变换（电压版本）
     * @param[in] ab_volt_in 两相静止坐标系输入
     * @param[out] dq_volt_out 两相旋转坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode park_volt(const AlphaBetaVolts& ab_volt_in, DQVolts& dq_volt_out) noexcept;

    /**
     * @brief 等幅值反Park变换（电压版本）
     * @param[in] dq_volt_in 两相旋转坐标系输入
     * @param[out] ab_volt_out 两相静止坐标系输出
     * @return 错误码
     */
    [[nodiscard]] common::ErrorCode inverse_park_volt(const DQVolts& dq_volt_in, AlphaBetaVolts& ab_volt_out) noexcept;
} // namespace robot_foc::algorithm

#endif
