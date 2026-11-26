/**
 * @file pid.hpp
 * @brief 分步式 PI、位置式 PID 和增量式 PID 控制算法接口。
 * @details 三类控制器均为纯算法模块，不依赖硬件或 RTOS。
 * - PiController：通过 preview、prepare_update 和 commit
 * - 分步更新积分，支持 commit_pair 双控制器成对提交。 * - PositionalPid：输出绝对控制量，支持输出限幅和积分限幅。
 * - IncrementalPid：输出控制增量，保存误差历史。
 * PiController 提供受控公共接口，通过来源及版本校验拒绝无效或过期的状态。
 * @note 数值运算采用 float；PI 接口通过 common::ErrorCode 报告业务错误。
 * @warning 实际采样周期应与配置或调参采用的采样周期一致。
 */

#ifndef ROBOT_FOC_ALGORITHM_PID_HPP
#define ROBOT_FOC_ALGORITHM_PID_HPP

#include <cstdint>

#include "units.hpp"
#include "error_code.hpp"

namespace robot_foc::algorithm
{
    /**
     * @brief PI 控制器配置参数集
     * @details 包含比例积分增益、采样周期与积分输出限幅，作为 configure 接口的入参。
     *
     * @var kp_ 比例增益
     * @var ki_ 积分增益
     * @var sample_time_ 采样周期，单位：秒
     * @var integral_min_ 积分输出下限
     * @var integral_max_ 积分输出上限
     */
    struct PiConfig final
    {
        float kp_ {0.0F};
        float ki_ {0.0F};
        common::Seconds sample_time_ {0.0F};
        float integral_min_ {0.0F};
        float integral_max_ {0.0F};
    };

    /**
     * @brief 通用分步式 PI 控制器。
     * @details 采用方向感知的条件积分策略；输入输出的物理单位由调用方约定。
     * 配置成功后，每次更新依次执行 preview、外部限幅、prepare_update、commit（或 commit_pair）。
     * preview 和 prepare_update 不修改控制器自身状态；提交成功后更新积分和版本。
     * 同一版本可生成多个预览或候选状态；成功提交、成功配置或复位后，旧版本全部失效。
     * @warning 调用方必须保证限幅结果与对应预览一致，并检查各阶段返回值。
     * @warning 本类不提供并发同步；同一实例的访问须由调用方同步，校验至写入期间不得有其他状态修改。
     * @warning 预览及候选状态不得跨越所属控制器的销毁与重建；有效性检查依赖版本号未回绕。
     */
    class PiController final
    {
    public:
        /**
         * @brief 候选积分状态。
         * @details 由 prepare_update 生成，保存候选积分、来源实例及生成时的版本。
         * 默认构造的对象无效；外部不可直接访问内部字段。
         */
        class PreparedState final
        {
        private:
            friend class PiController;
            // 下一周期候选积分输出值
            float next_integral_ {0.0F};
            // 生成该状态的非拥有型控制器指针，用于来源校验
            const PiController *owner_ {nullptr};
            // 生成时控制器版本号，防止跨版本误用
            std::uint64_t revision_ {0U};
        public:
            PreparedState() noexcept = default;
            ~PreparedState() noexcept = default;
        };

        /**
         * @brief PI 预览输出。
         * @details 由 preview 生成，保存控制误差、未限幅请求控制量、来源实例及版本。
         * 默认构造的对象无效；外部仅可通过 getter 读取请求控制量。
         */
        class PiPreview final
        {
        private:
            friend class PiController;
            // 控制误差 = setpoint - feedback
            float error_ {0.0F};
            // 比例项 + 已提交积分项求和得到的原始控制量（限幅前）
            float request_output_ {0.0F};
            // 生成该预览的非拥有型控制器指针
            const PiController *owner_ {nullptr};
            // 生成时控制器版本号
            std::uint64_t revision_ {0U};

        public:
            PiPreview() noexcept = default;
            ~PiPreview() noexcept = default;

            /**
             * @brief 按值读取未限幅的原始请求控制量。
             * @pre 仅在 preview 成功且该预览仍适用时使用。
             * @note 本函数不执行来源或版本校验。
             * @return 原始请求控制量。
             */
            [[nodiscard]] float get_request_output() const noexcept;
        };

        PiController() noexcept = default;
        ~PiController() noexcept = default;

        // 禁止拷贝与移动，避免控制器状态被无意复制导致的逻辑异常
        PiController(const PiController&) = delete;
        PiController& operator=(const PiController&) = delete;
        PiController(PiController&&) = delete;
        PiController& operator=(PiController&&) = delete;

        /**
         * @brief 校验并设置 PI 配置。
         * @details 成功时更新配置和离散积分系数，清零积分，标记就绪并递增版本。
         * 失败时配置、积分、就绪标志、离散积分系数及版本均保持不变。
         * @param[in] config PI 增益、采样周期及积分输出限幅。
         * @retval common::ErrorCode::Ok 配置成功。
         * @retval common::ErrorCode::ErrorInvalidParam 参数或 Ki*Ts 非有限、增益为负、采样周期非正或积分范围不包含零。
         * @retval common::ErrorCode::ErrorAlreadyInit 已初始化过
         */
        [[nodiscard]] common::ErrorCode configure(
            const PiConfig& config
        ) noexcept;

        /**
         * @brief 计算未限幅的原始请求控制量。
         * @details 不修改控制器内部状态；成功时绑定来源和当前版本，失败时使输出预览失效。
         * @param[in] setpoint 目标设定值。
         * @param[in] feedback 反馈测量值，与目标值单位一致。
         * @param[out] preview_out 预览输出
         * 失败时 owner_ 被清空，预览对象无效。
         * 其他成员不应读取。         
         * @retval common::ErrorCode::Ok 计算成功。
         * @retval common::ErrorCode::ErrorNotReady 控制器未配置。
         * @retval common::ErrorCode::ErrorInvalidParam 输入或计算结果非有限。
         */
        [[nodiscard]] common::ErrorCode preview(
            const float setpoint,
            const float feedback,
            PiPreview& preview_out
        ) const noexcept;

        /**
         * @brief 根据外部限幅结果生成候选积分状态。
         * @details 校验预览来源及版本，执行方向感知条件积分和积分限幅，不修改控制器内部状态。
         * 成功时绑定候选状态的来源和版本；失败时使输出候选状态失效。
         * @param[in] preview 当前控制器、当前版本的有效预览。
         * @param[in] limited_output 与当前预览对应的限幅后控制量，
         * 单位及坐标轴必须与 preview.request_output_ 一致。
         * @param[in] limit_flag 是否发生外部输出限幅。
         * @param[out] next_state 输出候选积分状态；
         * 失败时 owner_ 被清空，候选状态无效，
         * 其他成员不应读取。         
         * @retval common::ErrorCode::Ok 候选状态生成成功。
         * @retval common::ErrorCode::ErrorNotReady 控制器未配置。
         * @retval common::ErrorCode::ErrorInvalidParam 输入或计算结果非法。
         * @retval common::ErrorCode::ErrorUnsupported 预览来源或版本不匹配。
         */
        [[nodiscard]] common::ErrorCode prepare_update(
            const PiPreview& preview,
            float limited_output,
            bool limit_flag,
            PreparedState& next_state
        ) const noexcept;

        /**
         * @brief 校验并提交候选积分状态。
         * @details 检查配置状态、候选来源、版本及积分约束；成功时更新积分和版本。
         * 失败时控制器内部状态保持不变；无论成功或失败均不修改候选对象。
         * @param[in] state 待提交的候选状态。
         * @retval common::ErrorCode::Ok 提交成功。
         * @retval common::ErrorCode::ErrorNotReady 控制器未配置。
         * @retval common::ErrorCode::ErrorInvalidParam 积分非有限或超出配置范围。
         * @retval common::ErrorCode::ErrorUnsupported 来源或版本不匹配.
         */
        [[nodiscard]] common::ErrorCode commit(const PreparedState& state) noexcept;

        /**
         * @brief 成对提交两个 PI 控制器的候选积分状态。
         * @details 先检查实例是否不同，再校验两个候选状态，全部通过后才更新两者积分及版本。
         * 任意检查失败时两个控制器均保持原状态；不修改候选对象。
         * @param[in,out] first 第一个 PI 控制器。
         * @param[in] first_state 第一个控制器的候选状态。
         * @param[in,out] second 第二个 PI 控制器。
         * @param[in] second_state 第二个控制器的候选状态。
         * @retval common::ErrorCode::Ok 两个控制器均提交成功。
         * @retval common::ErrorCode::ErrorNotReady 控制器未配置。
         * @retval common::ErrorCode::ErrorInvalidParam  两个控制器为同一实例，或候选积分非法。
         * @retval common::ErrorCode::ErrorUnsupported 来源或版本不匹配.
         * @warning 调用期间不得有并发状态修改；本接口不提供并发同步或指令级原子性。
         */
        [[nodiscard]] static common::ErrorCode commit_pair(
            PiController& first,
            const PreparedState& first_state,
            PiController& second,
            const PreparedState& second_state
        ) noexcept;

        /**
         * @brief 复位积分状态。
         * @details 清零积分并递增版本，即使原积分为零也使旧预览和候选状态失效。
         * 保留配置、离散积分系数及就绪标志。
         */
        void reset() noexcept;

    private:
        // PI 控制器配置参数
        PiConfig piconfig_ {};
        // 已提交生效的积分输出项
        float integral_output_ {0.0F};
        // 配置就绪标志
        bool configured_ {false};
        // 预计算离散积分系数：ki * sample_time
        float ki_step_ {0.0F};
        // 当前状态版本，用于校验预览和候选状态是否过期
        std::uint64_t revision_ {0U};

        /**
         * @brief 校验候选积分状态，不修改任何状态。
         * @param[in] state 待校验的候选状态。
         * @retval common::ErrorCode::Ok 状态合法。
         * @retval common::ErrorCode::ErrorNotReady 控制器未配置。
         * @retval common::ErrorCode::ErrorInvalidParam 积分非有限或超出配置范围。
         * @retval common::ErrorCode::ErrorUnsupported 来源或版本不匹配.
         */
        [[nodiscard]] common::ErrorCode validate_prepared_state(
            const PreparedState& state
        ) const noexcept;

        /**
         * @brief 写入已校验的候选积分并递增版本。
         * @pre 候选状态已通过校验，且校验后控制器状态及候选对象未被修改。
         * @note 不重复校验，无业务失败返回路径，仅供内部提交操作调用。
         * @param[in] state 已校验的候选状态。
         */
        void apply_prepared_state(const PreparedState& state) noexcept;
    };

    /**
     * @brief 位置式 PID 控制器
     * @details
     * 输出绝对控制量 u(k)；支持输出限幅与积分限幅。
     * 适用于执行器接收绝对给定指令的场景，如速度环、位置环外环。
     */
    class PositionalPid final
    {
    private:
        float kp_ {0.0F}; ///< 比例系数
        float ki_ {0.0F}; ///< 积分系数
        float kd_ {0.0F}; ///< 微分系数

        float out_min_ {0.0F}; ///< 输出下限
        float out_max_ {0.0F}; ///< 输出上限
        float integral_min_ {0.0F}; ///< 积分项下限
        float integral_max_ {0.0F}; ///< 积分项上限

        float integral_ {0.0F}; ///< 积分累加值
        float last_err_ {0.0F}; ///< 上一拍误差 e(k-1)

    public:
        /**
         * @brief 构造位置式 PID 实例
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param out_min 输出下限
         * @param out_max 输出上限
         * @param integral_min 积分项下限
         * @param integral_max 积分项上限
         */
        PositionalPid(float kp, float ki, float kd, float out_min,
            float out_max, float integral_min, float integral_max
        ) noexcept;

        // 禁止拷贝与移动，防止状态意外复制
        PositionalPid(const PositionalPid&) = delete;
        PositionalPid& operator=(const PositionalPid&) = delete;
        PositionalPid(PositionalPid&&) = delete;
        PositionalPid& operator=(PositionalPid&&) = delete;

        /**
         * @brief 单周期 PID 计算
         * @param setpoint 目标给定值
         * @param feedback 反馈测量值
         * @return 经过输出限幅的绝对控制量 u(k)
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位 PID 动态状态
         * @details 清零积分累加值与上一拍误差；保留增益与限幅参数不变。
         */
        void reset() noexcept;

        /**
         * @brief 动态更新 PID 增益
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /** 
         * @brief 获取比例系数 
         * @return kp 
         */
        [[nodiscard]] float get_kp() const noexcept;

        /** 
         * @brief 获取积分系数
         * @return ki 
         */
        [[nodiscard]] float get_ki() const noexcept;

        /** 
         * @brief 获取微分系数 
         * @return kd 
         */
        [[nodiscard]] float get_kd() const noexcept;

        /** 
         * @brief 获取当前积分累加值 
         * @return integral 
         */
        [[nodiscard]] float get_integral() const noexcept;

        /** 
         * @brief 获取上一拍误差 
         * @return last_err 
         */
        [[nodiscard]] float get_last_err() const noexcept;
    };

    /**
     * @brief 增量式 PID 控制器
     * @details
     * 输出控制增量 Δu(k)；内部保存最近三拍误差 e(k), e(k-1), e(k-2)。
     * 适用于步进电机、阀门调节等增量执行场景，输出增量受配置的上下限约束。
     */
    class IncrementalPid final
    {
    private:
        float kp_ {0.0F}; ///< 比例系数
        float ki_ {0.0F}; ///< 积分系数
        float kd_ {0.0F}; ///< 微分系数

        float delta_out_min_ {0.0F}; ///< 增量输出下限
        float delta_out_max_ {0.0F}; ///< 增量输出上限

        float err1_ {0.0F}; ///< 当前误差 e(k)
        float err2_ {0.0F}; ///< 上一拍误差 e(k-1)
        float err3_ {0.0F}; ///< 上上拍误差 e(k-2)

    public:
        /**
         * @brief 构造增量式 PID 实例
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param delta_out_min 增量输出下限
         * @param delta_out_max 增量输出上限
         */
        IncrementalPid(float kp ,float ki, float kd,
            float delta_out_min, float delta_out_max
        ) noexcept;
        ~IncrementalPid() noexcept = default;

        // 禁止拷贝与移动，防止状态意外复制
        IncrementalPid(const IncrementalPid&) = delete;
        IncrementalPid& operator=(const IncrementalPid&) = delete;
        IncrementalPid(IncrementalPid&&) = delete;
        IncrementalPid& operator=(IncrementalPid&&) = delete;

        /**
         * @brief 单周期增量 PID 计算
         * @param setpoint 目标给定值
         * @param feedback 反馈测量值
         * @return 经过限幅的控制增量 Δu
         */
        [[nodiscard]] float calculate(float setpoint, float feedback) noexcept;

        /**
         * @brief 复位增量 PID 状态
         * @details 清空三拍误差历史；保留增益与增量限幅参数不变。
         */
        void reset() noexcept;

        /**
         * @brief 动态更新 PID 增益
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         */
        void set_param(float kp, float ki, float kd) noexcept;

        /** 
         * @brief 获取比例系数 
         * @return kp 
         */
        [[nodiscard]] float get_kp() const noexcept;

        /** 
         * @brief 获取积分系数 
         * @return ki 
         */
        [[nodiscard]] float get_ki() const noexcept;

        /** 
         * @brief 获取微分系数 
         * @return kd 
         */
        [[nodiscard]] float get_kd() const noexcept;

        /** 
         * @brief 获取当前误差 e(k) 
         * @return err1_ 
         */
        [[nodiscard]] float get_err1() const noexcept;

        /** 
         * @brief 获取上一拍误差 e(k-1) 
         * @return err2_ 
         */
        [[nodiscard]] float get_err2() const noexcept;

        /** 
         * @brief 获取上上拍误差 e(k-2) 
         * @return err3_ 
         */
        [[nodiscard]] float get_err3() const noexcept;
    };

} // namespace robot_foc::algorithm

#endif
