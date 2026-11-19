/**
 * @file pid.hpp
 * @brief 电流环PI、位置式PID和增量式PID控制算法接口。
 * @details
 * 本文件提供三个互相独立的纯算法控制器，不依赖硬件或RTOS：
 * - PiController：面向FOC电流环，采用preview/prepare_update/commit两阶段状态更新；
 * - PositionalPid：输出绝对控制量，支持输出限幅和积分限幅；
 * - IncrementalPid：输出控制增量，保存最近三次误差。
 *
 * PiController的prepare_update()和commit()为受限接口，只允许CurrentLoop编排器
 * 以及单元测试访问器调用。这样可以先完成d/q双轴候选状态计算，再统一提交，避免半更新。
 * @note 全部算法使用float运算，关闭异常时仍可使用；错误通过ErrorCode返回。
 * @warning 调用者必须保证控制器的实际调用周期与配置或调参时采用的采样周期一致。
 */

#ifndef ROBOT_FOC_ALGORITHM_PID_HPP
#define ROBOT_FOC_ALGORITHM_PID_HPP

#include "units.hpp"
#include "error_code.hpp"

namespace robot_foc::test
{
    class PiControllerTestAccess;
}

namespace robot_foc::algorithm
{
    /**
     * @brief PI控制器基础配置
     * @var kp_ 比例系数
     * @var ki_ 积分系数
     * @var sample_time_ 采样时间
     * @var integral_min_ 积分下限
     * @var integral_max_ 积分上限
     */
    struct PiConfig final
    {
        float kp_ {0.0F};
        float ki_ {0.0F};
        common::Seconds sample_time_ {0.0F};
        common::Volts integral_min_ {0.0F};
        common::Volts integral_max_ {0.0F};
    };
    
    /**
     * @brief PI控制前计算内容
     * @var error_ 控制误差
     * @var request_output_ 尚未进行共同限幅的原始控制量
     */
    struct PiPreview
    {
        common::Amperes error_ {0.0F};
        common::Volts request_output_ {0.0F};
    };
    
    class CurrentLoop;

    /**
     * @brief 适用于FOC电流环的分步式PI控制器。
     * @details
     * preview()只计算本周期请求电压，不修改积分状态；prepare_update()依据实际限幅结果
     * 生成下一积分候选值；commit()在所有相关计算均成功后提交候选值。
     *
     * 正常调用顺序：configure() → preview() → 外部矢量限幅 →
     * prepare_update() → commit()。CurrentLoop负责组织后两个私有步骤。
     */
    class PiController final
    {
    private:
        friend class CurrentLoop;
        friend class robot_foc::test::PiControllerTestAccess;

        // PI控制器基础配置
        PiConfig piconfig_ {};
        // 积分运行状态,已经提交的积分输出贡献
        common::Volts integral_output_ {0.0F};
        // 控制器配置完成标志位
        bool configured_ {false};
        // 离散积分系数 Ki * Ts，单位 V/A
        float ki_step_ {0.0F};
        /**
         * @brief 根据限幅结果准备PI下一周期状态
         * @details
         * 使用条件积分冻结策略计算下一积分贡献。函数只写入next_integral_out，
         * 不修改控制器内部状态；失败时next_integral_out保持不变。
         * @param[in] preview 本周期preview()输出的数据
         * @param[in] limited_output 本周期执行器实际采用的限幅后电压
         * @param[in] limit_flag 本周期是否发生矢量限幅
         * @param[out] next_integral_out 下一周期积分输出贡献
         * @retval common::ErrorCode::Ok 候选状态计算成功
         * @retval common::ErrorCode::ErrorNotReady 控制器尚未配置
         * @retval common::ErrorCode::ErrorInvalidParam 输入包含NaN/Inf或计算结果非法
         */
        [[nodiscard]] common::ErrorCode prepare_update(
            const PiPreview& preview,
            common::Volts limited_output,
            bool limit_flag,
            common::Volts& next_integral_out
        ) const noexcept;

        /**
         * @brief 提交并固化PI下一周期状态
         * @param[in] next_integral 已由prepare_update()校验的下一积分输出贡献
         * @pre next_integral必须来自同一PiController最近一次成功的prepare_update()调用。
         * @note 本函数为受信任的内部提交原语，不重复执行参数校验。
         */
        void commit(common::Volts next_integral) noexcept;

    public:
        PiController() noexcept = default;
        ~PiController() noexcept = default;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        PiController(const PiController&) = delete;
        PiController& operator=(const PiController&) = delete;
        PiController(PiController&&) = delete;
        PiController& operator=(PiController&&) = delete;
        
        /**
         * @brief 计算本周期未限幅的PI请求电压。
         * @details 请求电压等于比例输出与已提交积分输出之和；本函数不修改内部状态。
         * @param[in] setpoint 电流目标值，单位A
         * @param[in] feedback 电流反馈值，单位A
         * @param[out] preview_out 控制误差和未限幅请求电压；失败时保持不变
         * @retval common::ErrorCode::Ok 计算成功
         * @retval common::ErrorCode::ErrorNotReady 控制器尚未配置
         * @retval common::ErrorCode::ErrorInvalidParam 输入或计算结果包含NaN/Inf
         */
        [[nodiscard]] common::ErrorCode preview(
            const common::Amperes setpoint,
            const common::Amperes feedback,
            PiPreview& preview_out
        ) const noexcept;

        /**
         * @brief 校验并设置PI参数。
         * @details 配置成功时清零积分状态；配置失败时保留原配置和运行状态。
         * @param[in] config PI增益、采样周期和积分输出限幅
         * @retval common::ErrorCode::Ok 配置成功
         * @retval common::ErrorCode::ErrorInvalidParam 参数非有限、增益小于0、采样周期非正或限幅方向非法
         */
        [[nodiscard]] common::ErrorCode configure(const PiConfig& config) noexcept;
        
        /**
         * @brief 清零已提交的积分状态。
         * @note 保留configure()设置的全部参数和已配置状态。
         */
        void reset() noexcept;
    };
    
    /**
     * @brief 位置式Pid控制器
     */
    class PositionalPid final
    {
    private:
        float kp_{0.0F};  //比例系数
        float ki_{0.0F};  //积分系数
        float kd_{0.0F};  //微分系数

        float out_min_{0.0F};  //输出下限
        float out_max_{0.0F};  //输出上限
        float integral_min_{0.0F};  //积分项下限
        float integral_max_{0.0F};  //积分项上限

        float integral_{0.0F};  //积分累加值
        float last_err_{0.0F};  //上一次误差

    public:
        /**
         * @brief 构造位置式PID
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param out_min 输出下限
         * @param out_max 输出上限
         * @param integral_min 积分项下限
         * @param integral_max 积分项上限
         */
        PositionalPid(float kp, float ki, float kd, float out_min,
            float out_max, float integral_min, float integral_max) noexcept;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        PositionalPid(const PositionalPid&) = delete;
        PositionalPid& operator=(const PositionalPid&) = delete;
        PositionalPid(PositionalPid&&) = delete;
        PositionalPid& operator=(PositionalPid&&) = delete;

        /**
         * @brief 单次PID计算
         * @param setpoint 目标设定值
         * @param feedback 反馈测量值
         * @return 位置式控制输出，做输出限幅
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位，清零积分项与历史误差
         */
        void reset() noexcept;

        /**
         * @brief 动态修改Pid参数
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /**
         * @brief 获取kp
         * @return kp_
         */
        [[nodiscard]] float get_kp() const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki() const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd() const noexcept;

        /**
         * @brief 获取积分累积值
         * @return integral_
         */
        [[nodiscard]] float get_integral() const noexcept;

        /**
         * @brief 获取上一次误差
         * @return last_err_
         */
        [[nodiscard]] float get_last_err() const noexcept;
    };

    /**
     * @brief 增量式Pid控制器
     */
    class IncrementalPid final
    {
    private:
        float kp_{0.0F};  //比例系数
        float ki_{0.0F};  //积分系数
        float kd_{0.0F};  //微分系数

        float delta_out_min_{0.0F};  //增量输出下限
        float delta_out_max_{0.0F};  //增量输出上限

        float err1_{0.0F};  //e(k)
        float err2_{0.0F};  //e(k-1)
        float err3_{0.0F};  //e(k-2)

    public:
        /**
         * @brief 构造增量式PID
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param delta_out_min 输出增量下限
         * @param delta_out_max 输出增量上限
         */
        IncrementalPid(float kp ,float ki, float kd,
            float delta_out_min, float delta_out_max
        ) noexcept;
        ~IncrementalPid() noexcept = default;

        // 禁止拷贝、禁止移动，防止无意复制控制器状态
        IncrementalPid(const IncrementalPid&) = delete;
        IncrementalPid& operator=(const IncrementalPid&) = delete;
        IncrementalPid(IncrementalPid&&) = delete;
        IncrementalPid& operator=(IncrementalPid&&) = delete;

        /**
         * @brief 单次PID计算
         * @param setpoint 目标设定值
         * @param feedback 反馈测量值
         * @return 控制增量Δu，做输出限幅
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位，err1,err2,err3清零
         */
        void reset() noexcept;

        /**
         * @brief 动态修改Pid参数
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /**
         * @brief 获取kp
         * @return kp_
         */
        [[nodiscard]] float get_kp() const noexcept;

        /**
         * @brief 获取ki
         * @return ki_
         */
        [[nodiscard]] float get_ki() const noexcept;

        /**
         * @brief 获取kd
         * @return kd_
         */
        [[nodiscard]] float get_kd() const noexcept;

        /**
         * @brief 获取err1
         * @return err1_
         */
        [[nodiscard]] float get_err1() const noexcept;

        /**
         * @brief 获取err2
         * @return err2_
         */
        [[nodiscard]] float get_err2() const noexcept;

        /**
         * @brief 获取err3
         * @return err3_
         */
        [[nodiscard]] float get_err3() const noexcept;



    };

} // namespace robot_foc::algorithm

#endif
