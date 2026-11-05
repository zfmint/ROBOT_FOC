/**
 * @file foc_transform.hpp
 * @brief FOC Clarke / Park / InverseClarke / InversePark坐标变换
 * @details
 * 采用【等幅值 Clarke】；所有角度统一为电角度，单位 rad；
 * 电流:A，电压:V；全部为无状态纯数学变换，无内部缓存。
 *
 * 变换清单：
 *  1. Clarke: 三相/两相 → 静止两相正交
 *  2. InverseClarke 静止两相正交 → 三相
 *  3. Park:   αβ → dq  同步旋转dq
 *  4. InversePark: dq → αβ
 *
 * @author Robot_FOC Project
 * @note
 *  - 角度必须是【电角度】，不是机械角度
 *  - 角度范围不限，内部直接送入sin/cos，由三角函数处理周期
 *  - Clarke 默认等幅值，不是等功率
 */

#ifndef ROBOT_FOC_ALGORITHM_FOC_TRANSFORM_HPP
#define ROBOT_FOC_ALGORITHM_FOC_TRANSFORM_HPP

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"

namespace robot_foc::algorithm
{
    /**
     * @brief FOC坐标变换静态工具类
     * @details 全部为无状态静态数学函数，不可实例化，禁止拷贝移动
     */
    class FocTransform
    {
    private:

    public:
        FocTransform() noexcept = delete;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        FocTransform(const FocTransform&) = delete;
        FocTransform& operator=(const FocTransform&) = delete;
        FocTransform(FocTransform&&) = delete;
        FocTransform& operator=(FocTransform&&) = delete;

        /**
         * @brief 等幅值Clarke变换：三相ABC电流 → αβ静止两相电流
         * @param[in] abc_current_in 三相ABC电流输入
         * @param[out] alphabeta_current_out αβ两相静止坐标系电流输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode clarke(const common::ABCCurrent& abc_current_in, common::AlphaBetaCurrent& alphabeta_current_out) noexcept;
        /**
         * @brief 等幅值Clarke变换：三相ABC电压 → αβ静止两相电压
         * @param[in] abc_volt_in 三相ABC电压输入
         * @param[out] alphabeta_volt_out αβ两相静止坐标系电压输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode clarke(const common::ABCVoltage& abc_volt_in, common::AlphaBetaVoltage& alphabeta_volt_out) noexcept;
        /**
         * @brief 等幅值Clarke变换：两相AB电流 → αβ静止两相电流（内部自动计算C = -A‑B）
         * @param[in] ab_current_in 两相AB电流输入
         * @param[out] alphabeta_current_out αβ两相静止坐标系电流输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode clarke(const common::ABCurrent& ab_current_in, common::AlphaBetaCurrent& alphabeta_current_out) noexcept;
        /**
         * @brief 等幅值Clarke变换：两相AB电压 → αβ静止两相电压（内部自动计算C = -A‑B）
         * @param[in] ab_volt_in 两相AB电压输入
         * @param[out] alphabeta_volt_out αβ两相静止坐标系电压输出
         * @return 错误码
         */        
        [[nodiscard]] static common::ErrorCode clarke(const common::ABVoltage& ab_volt_in, common::AlphaBetaVoltage& alphabeta_volt_out) noexcept;

        /**
         * @brief 等幅值逆Clarke变换：αβ静止两相电流 → 三相ABC电流
         * @param[in] alphabeta_current_in αβ两相静止坐标系电流输入
         * @param[out] abc_current_out 三相ABC电流输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode inverse_clarke(const common::AlphaBetaCurrent& alphabeta_current_in, common::ABCCurrent& abc_current_out) noexcept;
        /**
         * @brief 等幅值逆Clarke变换：αβ静止两相电压 → 三相ABC电压
         * @param[in] alphabeta_volt_in αβ两相静止坐标系电压输入
         * @param[out] abc_volt_out 三相ABC电压输出
         * @return 错误码
         */        
        [[nodiscard]] static common::ErrorCode inverse_clarke(const common::AlphaBetaVoltage& alphabeta_volt_in, common::ABCVoltage& abc_volt_out) noexcept;

        /**
         * @brief Park变换：αβ静止两相电流 → dq旋转坐标系电流
         * @param[in] alphabeta_current_in αβ两相静止坐标系电流输入
         * @param[in] theta_e 转子电角度 ElectricalRad
         * @param[out] dq_current_out d‑q旋转坐标系电流输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode park(const common::AlphaBetaCurrent& alphabeta_current_in, const common::ElectricalRad theta_e, common::DQCurrent& dq_current_out) noexcept;
        /**
         * @brief Park变换：αβ静止两相电压 → dq旋转坐标系电压
         * @param[in] alphabeta_volt_in αβ两相静止坐标系电压输入
         * @param[in] theta_e 转子电角度 ElectricalRad
         * @param[out] dq_volt_out d‑q旋转坐标系电压输出
         * @return 错误码
         */        
        [[nodiscard]] static common::ErrorCode park(const common::AlphaBetaVoltage& alphabeta_volt_in, const common::ElectricalRad theta_e, common::DQVoltage& dq_volt_out) noexcept;

        /**
         * @brief 逆Park变换：dq旋转坐标系电流 → αβ静止两相电流
         * @param[in] dq_current_in d‑q旋转坐标系电流输入
         * @param[in] theta_e 转子电角度 ElectricalRad
         * @param[out] alphabeta_current_out αβ两相静止坐标系电流输出
         * @return 错误码
         */
        [[nodiscard]] static common::ErrorCode inverse_park(const common::DQCurrent& dq_current_in, const common::ElectricalRad theta_e, common::AlphaBetaCurrent& alphabeta_current_out) noexcept;
        /**
         * @brief 逆Park变换：dq旋转坐标系电压 → αβ静止两相电压
         * @param[in] dq_volt_in d‑q旋转坐标系电压输入
         * @param[in] theta_e 转子电角度 ElectricalRad
         * @param[out] alphabeta_volt_out αβ两相静止坐标系电压输出
         * @return 错误码
         */        
        [[nodiscard]] static common::ErrorCode inverse_park(const common::DQVoltage& dq_volt_in, const common::ElectricalRad theta_e, common::AlphaBetaVoltage& alphabeta_volt_out) noexcept;
    };
} // namespace robot_foc::algorithm

#endif
