/**
 * @file control_type.hpp
 * @brief 基础物理量类型别名，统一全项目单位
 * @note 仅做别名，不做强单位编译期检查；后续可升级为带单位包装类型；
 *       每个类型注释明确物理含义与单位，禁止裸float/int表达物理量。
 */

#ifndef ROBOT_FOC_CONTROL_TYPE_HPP
#define ROBOT_FOC_CONTROL_TYPE_HPP

#include "units.hpp"

namespace robot_foc::common
{
    /**
     * @brief 三相电流结构体 A/B/C
     */
    struct ABCCurrent
    {
        common::Amperes a_current_ = 0.0F;
        common::Amperes b_current_ = 0.0F;
        common::Amperes c_current_ = 0.0F;
    };

    /**
     * @brief 三相电压结构体 A/B/C
     */    struct ABCVoltage
    {
        common::Volts a_volt_ = 0.0F;
        common::Volts b_volt_ = 0.0F;
        common::Volts c_volt_ = 0.0F;
    };

    /**
     * @brief 两相AB电流结构体，仅A、B两相；C相由C = -A‑B推导
     */
    struct ABCurrent
    {
        common::Amperes a_current_ = 0.0F;
        common::Amperes b_current_ = 0.0F;
    };
    
    /**
     * @brief 两相AB电压结构体，仅A、B两相；C相由C = -A‑B推导
     */
    struct ABVoltage
    {
        common::Volts a_volt_ = 0.0F;
        common::Volts b_volt_ = 0.0F;
    };

    /**
     * @brief α‑β静止两相正交电流
     */
    struct AlphaBetaCurrent
    {
        common::Amperes alpha_current_ = 0.0F;
        common::Amperes beta_current_ = 0.0F;
    };

    /**
     * @brief d‑q旋转坐标系电流
     */
    struct DQCurrent
    {
        common::Amperes d_current_ = 0.0F;
        common::Amperes q_current_ = 0.0F;
    };
    
    /**
     * @brief α‑β静止两相正交电压
     */
    struct AlphaBetaVoltage
    {
        common::Volts alpha_volt_ = 0.0F;
        common::Volts beta_volt_ = 0.0F;
    };

    /**
     * @brief d‑q旋转坐标系电压
     */
    struct DQVoltage
    {
        common::Volts d_volt_ = 0.0F;
        common::Volts q_volt_ = 0.0F;
    };
} // namespace robot_foc::common

#endif
