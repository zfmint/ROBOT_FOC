/**
 * @file units.hpp
 * @brief 基础物理量类型别名，统一全项目单位
 * @note 仅做别名，不做强单位编译期检查；后续可升级为带单位包装类型；
 *       每个类型注释明确物理含义与单位，禁止裸float/int表达物理量。
 */
#ifndef ROBOT_FOC_COMMON_UNITS_HPP
#define ROBOT_FOC_COMMON_UNITS_HPP

#include <cstdint>

namespace robot_foc::common
{
    using Seconds       =   float;          ///时间，单位：秒（s）
    using Milliseconds  =   std::uint32_t;  ///时间，单位：毫秒（ms）
    using Volts         =   float;          ///电压，单位：伏特（V）
    using Amperes        =   float;          ///电流，单位：安培（A）
    using Radians        =   float;          ///角度，单位：弧度（Rad）
    using Rpm           =   float;          ///转速，单位：转每分钟（RPM）
} // namespace robot_foc::common



#endif // ROBOT_FOC_COMMON_UNITS_HPP
