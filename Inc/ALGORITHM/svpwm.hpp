/**
 * @file svpwm.hpp
 * @brief 七段式SVPWM空间矢量调制，等幅值体系，无状态纯数学模块
 * @details
 * 输入 αβ静止坐标系电压，输出三相PWM占空比（0.0 ~ 1.0）
 * 支持扇区判断、矢量时间计算、过调制限幅保护
 * @note
 * 1. 输出占空比，后续交给底层PWM驱动转换为CCR值
 * 2. 母线电压Vbus为输入参数，支持动态修改母线
 */
#ifndef ROBOT_FOC_ALGORITHM_SVPWM_HPP
#define ROBOT_FOC_ALGORITHM_SVPWM_HPP

#include "units.hpp"
#include "error_code.hpp"

namespace robot_foc::algorithm
{
    
} // namespace robot_foc::algorithm

#endif
