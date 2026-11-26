/**
 * @file test_current_loop.cpp
 * @brief CurrentLoop FOC dq电流内环单元测试
 * @details
 * 黑盒测试CurrentLoop对外公开API；底层依赖PiController、FocTransform、VoltageLimit、Svpwm均已完成单元测试。
 * 内部流水线【真实执行顺序】：
 * 1. preview：Id/Iq预计算原始Vd/Vq
 * 2. inverse_park → VoltageLimit六边形矢量限幅
 * 3. park回dq（使用限幅后αβ电压，用于抗饱和prepare_update）
 * 4. prepare_update 根据限幅状态计算下一积分状态
 * 5. svpwm_calculate 计算三相占空比【SVPWM成功返回Ok之后】
 * 6. commit_pair 成对固化PI积分（只有变换/限幅/SVPWM全部成功，才提交两轴状态）
 *
 * 测试覆盖项：
 * 1. configure正常配置、重复配置返回ErrorAlreadyInit
 * 2. 非法PI参数：kp负数、sample_time为0，全新实例返回ErrorInvalidParam
 * 3. 未configure调用calculate返回ErrorNotReady，输出缓冲区不篡改脏值
 * 4. reset复位有效性校验：对比全新同配置对象首次输出，验证PI积分真正清零
 * 5. 完整FOC闭环仿真验收项：逆SVPWM还原UαUβ + mock::AlphaBetaFirstOrderPlant电机模型 + Park变换回Id/Iq，完成整体链路电流跟踪
 * 6. 电压矢量饱和工况与is_voltage_limited_标志测试
 * 7. NaN非法输入保护，输出缓冲区保留脏值
 * 8. 底层模块错误码向上透传
 * 9. 第二轴候选状态生成失败时，第一轴候选不得单独提交
 *
 * @note
 * 1. CurrentLoop内部使用PiController::commit_pair成对提交；本文件仅通过calculate黑盒验证事务语义。
 * 2. 本文件仅属于PC-test测试工程，禁止加入robot_hal_core业务库；业务代码禁止include测试头文件。
 * 3. 浮点数比较统一使用test_helper::nearly_equal。
 * 4. duty_to_alpha_beta：仅单元测试内部简易逆SVPWM解析工具，不属于被控对象模型，业务代码不使用。
 */
#include <cmath>
#include <limits>
#include "ALGORITHM/current_loop.hpp"
#include "ALGORITHM/pid.hpp"
#include "ALGORITHM/foc_transform.hpp"
#include "mock_plant.hpp"
#include "test_helper.hpp"
#include "error_code.hpp"

namespace
{
    /**
     * @brief 根据三相桥臂占空比和母线电压还原等幅值αβ电压
     * @details 先还原三相桥臂电压，再复用生产算法的三相等幅值Clarke变换；
     * Clarke变换会自动消除三相桥臂电压中的公共模分量。
     * @note 仅用于单元测试仿真，业务代码禁止使用
     * @return Clarke变换结果
     */
    [[nodiscard]] robot_foc::common::ErrorCode duty_to_alpha_beta(
        const robot_foc::algorithm::SvpwmDuty& duty,
        robot_foc::common::Volts v_bus,
        robot_foc::common::AlphaBetaVoltage& alpha_beta_out) noexcept
    {
        const float v_bus_value = static_cast<float>(v_bus);
        robot_foc::common::ABCVoltage phase_voltage {};
        phase_voltage.a_volt_ = robot_foc::common::Volts{duty.a_duty_ * v_bus_value};
        phase_voltage.b_volt_ = robot_foc::common::Volts{duty.b_duty_ * v_bus_value};
        phase_voltage.c_volt_ = robot_foc::common::Volts{duty.c_duty_ * v_bus_value};

        return robot_foc::algorithm::FocTransform::clarke(
            phase_voltage,
            alpha_beta_out
        );
    }
}

void test_current_loop()
{
    using namespace robot_foc::algorithm;
    using namespace robot_foc::common;
    using namespace robot_foc::test::mock;

    constexpr float steady_tol{0.3F};

    // -------- 测试1：configure正常配置 + 重复配置保护 --------
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 1.0F;
        cfg_id.ki_ = 50.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -10.0F;
        cfg_id.integral_max_ = 10.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg1 = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg1 == ErrorCode::Ok, "current_loop first configure ok");

        auto ret_cfg2 = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg2 == ErrorCode::ErrorAlreadyInit, "current_loop duplicate configure reject");
    }

    // -------- 测试2-1：非法PI参数 kp为负数 --------
    {
        CurrentLoop bad_loop{};
        PiConfig bad_cfg_id{};
        bad_cfg_id.kp_ = -5.0F;
        bad_cfg_id.ki_ = 50.0F;
        bad_cfg_id.sample_time_ = Seconds{0.0001F};
        bad_cfg_id.integral_min_ = -10.0F;
        bad_cfg_id.integral_max_ = 10.0F;
        PiConfig bad_cfg_iq = bad_cfg_id;

        auto ret_bad = bad_loop.configure(bad_cfg_id, bad_cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_bad == ErrorCode::ErrorInvalidParam, "current_loop bad pi kp negative reject");
    }

    // -------- 测试2-2：非法PI参数 sample_time等于0 --------
    {
        CurrentLoop bad_loop{};
        PiConfig bad_cfg_id{};
        bad_cfg_id.kp_ = 1.0F;
        bad_cfg_id.ki_ = 50.0F;
        bad_cfg_id.sample_time_ = Seconds{0.0F};
        bad_cfg_id.integral_min_ = -10.0F;
        bad_cfg_id.integral_max_ = 10.0F;
        PiConfig bad_cfg_iq = bad_cfg_id;

        auto ret_bad = bad_loop.configure(bad_cfg_id, bad_cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_bad == ErrorCode::ErrorInvalidParam, "current_loop bad pi sample_time zero reject");
    }

    // -------- 测试2-3：第二轴配置失败后保持未就绪，并允许重新配置 --------
    {
        CurrentLoop loop{};
        PiConfig valid_cfg{};
        valid_cfg.kp_ = 1.0F;
        valid_cfg.ki_ = 50.0F;
        valid_cfg.sample_time_ = Seconds{0.0001F};
        valid_cfg.integral_min_ = -10.0F;
        valid_cfg.integral_max_ = 10.0F;

        PiConfig invalid_iq_cfg = valid_cfg;
        invalid_iq_cfg.ki_ = -1.0F;
        ROBOT_FOC_TEST_EXPECT(
            loop.configure(valid_cfg, invalid_iq_cfg) == ErrorCode::ErrorInvalidParam,
            "current_loop second-axis invalid configure rejected"
        );

        CurrentLoopInput input{};
        CurrentLoopOutput output{};
        ROBOT_FOC_TEST_EXPECT(
            loop.calculate(input, output) == ErrorCode::ErrorNotReady,
            "current_loop remains not ready after second-axis configure failure"
        );
        ROBOT_FOC_TEST_EXPECT(
            loop.configure(valid_cfg, valid_cfg) == ErrorCode::Ok,
            "current_loop retry configure after second-axis failure succeeds"
        );
    }

    // -------- 测试3：未configure调用calculate，返回NotReady，输出保留脏值 --------
    {
        CurrentLoop loop{};
        CurrentLoopInput input{};
        CurrentLoopOutput output{};
        output.duty_.a_duty_ = 111.0F;
        output.duty_.b_duty_ = 222.0F;
        output.duty_.c_duty_ = 333.0F;
        output.is_voltage_limited_ = true;

        auto ret_calc = loop.calculate(input, output);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::ErrorNotReady, "current_loop calculate before configure return not ready");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.a_duty_, 111.0F), "current_loop not-ready preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.b_duty_, 222.0F), "current_loop not-ready preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.c_duty_, 333.0F), "current_loop not-ready preserve dutyC");
        ROBOT_FOC_TEST_EXPECT(output.is_voltage_limited_, "current_loop not-ready preserve limit flag");
    }

    // -------- 测试4：reset复位有效性校验，验证积分真正清零 --------
    // 逻辑：对象A累积积分后reset；对象B全新同配置；对比首次输出duty
    {
        PiConfig cfg{};
        cfg.kp_ = 2.0F;
        cfg.ki_ = 100.0F;
        cfg.sample_time_ = Seconds{0.0001F};
        cfg.integral_min_ = -10.0F;
        cfg.integral_max_ = 10.0F;

        CurrentLoop loop_a{};
        ROBOT_FOC_TEST_EXPECT(loop_a.configure(cfg, cfg) == ErrorCode::Ok, "current_loop reset-test A configure ok");

        CurrentLoopInput input{};
        input.current_reference_.d_current_ = Amperes{2.0F};
        input.current_reference_.q_current_ = Amperes{2.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};
        CurrentLoopOutput dummy_output{};

        bool accumulation_ok {true};
        for (int i = 0; i < 20; ++i)
        {
            const auto ret = loop_a.calculate(input, dummy_output);
            accumulation_ok = accumulation_ok && (ret == ErrorCode::Ok);
        }
        ROBOT_FOC_TEST_EXPECT(accumulation_ok, "current_loop reset-test accumulation ok");

        loop_a.reset();
        CurrentLoopOutput output_after_reset{};
        auto ret_a = loop_a.calculate(input, output_after_reset);
        ROBOT_FOC_TEST_EXPECT(ret_a == ErrorCode::Ok, "current_loop after-reset calculate ok");

        CurrentLoop loop_b{};
        ROBOT_FOC_TEST_EXPECT(loop_b.configure(cfg, cfg) == ErrorCode::Ok, "current_loop reset-test B configure ok");
        CurrentLoopOutput output_new_obj{};
        auto ret_b = loop_b.calculate(input, output_new_obj);
        ROBOT_FOC_TEST_EXPECT(ret_b == ErrorCode::Ok, "current_loop new-obj calculate ok");

        ROBOT_FOC_TEST_EXPECT(nearly_equal(output_after_reset.duty_.a_duty_, output_new_obj.duty_.a_duty_, 0.01F),
            "current_loop reset clear integral check A");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output_after_reset.duty_.b_duty_, output_new_obj.duty_.b_duty_, 0.01F),
            "current_loop reset clear integral check B");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output_after_reset.duty_.c_duty_, output_new_obj.duty_.c_duty_, 0.01F),
            "current_loop reset clear integral check C");
        ROBOT_FOC_TEST_EXPECT(
            !output_after_reset.is_voltage_limited_ && !output_new_obj.is_voltage_limited_,
            "current_loop non-saturated result clears limit flag"
        );
    }

    // -------- 测试5：【验收项】CurrentLoop完整闭环电流跟踪仿真 --------
    // 链路：calculate输出duty → 逆SVPWM得到UαUβ → AlphaBetaFirstOrderPlant电机模型得到IαIβ → Park变换得到IdIq反馈 → 回灌输入
    {
        constexpr float ts = 0.0001F;
        constexpr float duty_tolerance = 1.0e-4F;
        CurrentLoop loop{};
        AlphaBetaFirstOrderPlant plant(ts, 0.08F);
        plant.reset();

        PiConfig cfg_id{};
        cfg_id.kp_ = 1.8F;
        cfg_id.ki_ = 120.0F;
        cfg_id.sample_time_ = Seconds{ts};
        cfg_id.integral_min_ = -12.0F;
        cfg_id.integral_max_ = 12.0F;
        PiConfig cfg_iq = cfg_id;

        ROBOT_FOC_TEST_EXPECT(loop.configure(cfg_id, cfg_iq) == ErrorCode::Ok, "current_loop closed-loop configure ok");

        const float ref_d = 1.0F;
        const float ref_q = 2.5F;
        CurrentLoopInput input{};
        CurrentLoopOutput output{};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};

        // 先验证测试侧占空比反算工具与项目SVPWM定义互为近似逆变换。
        AlphaBetaVoltage round_trip_input{};
        round_trip_input.alpha_volt_ = Volts{2.0F};
        round_trip_input.beta_volt_ = Volts{1.0F};
        SvpwmDuty round_trip_duty{};
        const auto round_trip_ret = Svpwm::svpwm_calculate(
            round_trip_input,
            input.v_bus_,
            round_trip_duty
        );
        ROBOT_FOC_TEST_EXPECT(
            round_trip_ret == ErrorCode::Ok,
            "current_loop duty inverse round-trip svpwm ok"
        );

        AlphaBetaVoltage round_trip_output{};
        const auto inverse_ret = duty_to_alpha_beta(
            round_trip_duty,
            input.v_bus_,
            round_trip_output
        );
        ROBOT_FOC_TEST_EXPECT(
            inverse_ret == ErrorCode::Ok,
            "current_loop duty inverse clarke succeeds"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(round_trip_output.alpha_volt_, round_trip_input.alpha_volt_, 1.0e-3F),
            "current_loop duty inverse restores alpha voltage"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(round_trip_output.beta_volt_, round_trip_input.beta_volt_, 1.0e-3F),
            "current_loop duty inverse restores beta voltage"
        );

        bool calculate_ok {true};
        bool duty_bounds_ok {true};
        bool voltage_limit_flag_clear {true};
        bool duty_inverse_ok {true};
        bool park_ok {true};
        for (int step = 0; step < 2500; ++step)
        {
            input.current_reference_.d_current_ = Amperes{ref_d};
            input.current_reference_.q_current_ = Amperes{ref_q};

            const auto ret = loop.calculate(input, output);
            if (ret != ErrorCode::Ok)
            {
                calculate_ok = false;
                break;
            }

            duty_bounds_ok = duty_bounds_ok
                && (output.duty_.a_duty_ >= -duty_tolerance)
                && (output.duty_.a_duty_ <= 1.0F + duty_tolerance)
                && (output.duty_.b_duty_ >= -duty_tolerance)
                && (output.duty_.b_duty_ <= 1.0F + duty_tolerance)
                && (output.duty_.c_duty_ >= -duty_tolerance)
                && (output.duty_.c_duty_ <= 1.0F + duty_tolerance);
            voltage_limit_flag_clear = voltage_limit_flag_clear
                && !output.is_voltage_limited_;

            AlphaBetaVoltage u_ab{};
            const auto inverse_step_ret = duty_to_alpha_beta(output.duty_, input.v_bus_, u_ab);
            if (inverse_step_ret != ErrorCode::Ok)
            {
                duty_inverse_ok = false;
                break;
            }

            AlphaBetaCurrent i_ab = plant.update(u_ab);

            DQCurrent i_dq{};
            const auto park_ret = FocTransform::park(i_ab, input.electrical_angle_, i_dq);
            if (park_ret != ErrorCode::Ok)
            {
                park_ok = false;
                break;
            }

            input.current_feedback_.d_current_ = i_dq.d_current_;
            input.current_feedback_.q_current_ = i_dq.q_current_;
        }

        ROBOT_FOC_TEST_EXPECT(calculate_ok, "current_loop closed-loop calculations succeed");
        ROBOT_FOC_TEST_EXPECT(duty_bounds_ok, "current_loop closed-loop duty remains in range");
        ROBOT_FOC_TEST_EXPECT(
            voltage_limit_flag_clear,
            "current_loop closed-loop remains outside voltage saturation"
        );
        ROBOT_FOC_TEST_EXPECT(duty_inverse_ok, "current_loop closed-loop duty inverse succeeds");
        ROBOT_FOC_TEST_EXPECT(park_ok, "current_loop closed-loop feedback park succeeds");

        const float fb_d = static_cast<float>(input.current_feedback_.d_current_);
        const float fb_q = static_cast<float>(input.current_feedback_.q_current_);
        ROBOT_FOC_TEST_EXPECT(nearly_equal(fb_d, ref_d, steady_tol), "current_loop closed-loop d-axis track ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(fb_q, ref_q, steady_tol), "current_loop closed-loop q-axis track ok");
    }

    // -------- 测试6：电压矢量饱和冒烟测试，低压母线触发六边形限幅 --------
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 5.0F;
        cfg_id.ki_ = 500.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -15.0F;
        cfg_id.integral_max_ = 15.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "current_loop anti-windup configure ok");

        CurrentLoopInput input{};
        input.v_bus_ = Volts{8.0F};
        input.current_reference_.d_current_ = Amperes{0.0F};
        input.current_reference_.q_current_ = Amperes{8.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        CurrentLoopOutput output{};

        bool saturation_ok {true};
        bool duty_bounds_ok {true};
        bool limit_flag_ok {true};
        for (int i = 0; i < 50; ++i)
        {
            const auto ret = loop.calculate(input, output);
            saturation_ok = saturation_ok && (ret == ErrorCode::Ok);
            duty_bounds_ok = duty_bounds_ok
                && (output.duty_.a_duty_ >= 0.0F) && (output.duty_.a_duty_ <= 1.0F)
                && (output.duty_.b_duty_ >= 0.0F) && (output.duty_.b_duty_ <= 1.0F)
                && (output.duty_.c_duty_ >= 0.0F) && (output.duty_.c_duty_ <= 1.0F);
            limit_flag_ok = limit_flag_ok && output.is_voltage_limited_;
        }
        ROBOT_FOC_TEST_EXPECT(saturation_ok, "current_loop saturation calculations succeed");
        ROBOT_FOC_TEST_EXPECT(duty_bounds_ok, "current_loop saturation duty remains in range");
        ROBOT_FOC_TEST_EXPECT(limit_flag_ok, "current_loop saturation reports voltage limited");
    }

    // -------- 测试7：NaN非法输入保护，输出duty保留脏值 --------
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 1.0F;
        cfg_id.ki_ = 50.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -10.0F;
        cfg_id.integral_max_ = 10.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "current_loop nan-test configure ok");

        CurrentLoopOutput output{};
        output.duty_.a_duty_ = 55.0F;
        output.duty_.b_duty_ = 66.0F;
        output.duty_.c_duty_ = 77.0F;
        output.is_voltage_limited_ = true;

        CurrentLoopInput input{};
        input.current_reference_.d_current_ = Amperes{NAN};
        input.current_reference_.q_current_ = Amperes{1.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};

        auto ret_nan = loop.calculate(input, output);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "current_loop nan input reject");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.a_duty_, 55.0F), "current_loop nan preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.b_duty_, 66.0F), "current_loop nan preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.c_duty_, 77.0F), "current_loop nan preserve dutyC");
        ROBOT_FOC_TEST_EXPECT(output.is_voltage_limited_, "current_loop nan preserve limit flag");
    }

    // -------- 测试8：零母线电压，电压限幅错误向上透传 --------
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 1.0F;
        cfg_id.ki_ = 50.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -10.0F;
        cfg_id.integral_max_ = 10.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "current_loop err-pass configure ok");

        CurrentLoopInput input{};
        CurrentLoopOutput output{};
        output.duty_.a_duty_ = 10.0F;
        output.duty_.b_duty_ = 20.0F;
        output.duty_.c_duty_ = 30.0F;
        output.is_voltage_limited_ = true;

        input.v_bus_ = Volts{0.0F};
        input.current_reference_.d_current_ = Amperes{1.0F};
        input.current_reference_.q_current_ = Amperes{1.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};

        auto ret_zero_vbus = loop.calculate(input, output);
        ROBOT_FOC_TEST_EXPECT(ret_zero_vbus == ErrorCode::ErrorInvalidParam, "current_loop pass-through voltage-limit error");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.a_duty_,10.0F), "current_loop zero-vbus preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.b_duty_,20.0F), "current_loop zero-vbus preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(output.duty_.c_duty_,30.0F), "current_loop zero-vbus preserve dutyC");
        ROBOT_FOC_TEST_EXPECT(output.is_voltage_limited_, "current_loop zero-vbus preserve limit flag");
    }

    // -------- 测试9：第二轴候选生成失败时，第一轴候选不得单独提交 --------
    {
        CurrentLoop loop{};
        CurrentLoop baseline{};

        PiConfig id_cfg{};
        id_cfg.kp_ = 0.0F;
        id_cfg.ki_ = 1.0F;
        id_cfg.sample_time_ = Seconds{1.0F};
        id_cfg.integral_min_ = -10.0F;
        id_cfg.integral_max_ = 10.0F;

        const float maximum = std::numeric_limits<float>::max();
        PiConfig iq_cfg{};
        iq_cfg.kp_ = 0.0F;
        iq_cfg.ki_ = maximum;
        iq_cfg.sample_time_ = Seconds{1.0F};
        iq_cfg.integral_min_ = -maximum;
        iq_cfg.integral_max_ = maximum;

        ROBOT_FOC_TEST_ASSERT(
            loop.configure(id_cfg, iq_cfg) == ErrorCode::Ok,
            "current_loop transaction configure succeeds"
        );
        ROBOT_FOC_TEST_ASSERT(
            baseline.configure(id_cfg, iq_cfg) == ErrorCode::Ok,
            "current_loop transaction baseline configure succeeds"
        );

        CurrentLoopInput failing_input{};
        failing_input.current_reference_.d_current_ = Amperes{1.0F};
        failing_input.current_reference_.q_current_ = Amperes{2.0F};
        failing_input.current_feedback_.d_current_ = Amperes{0.0F};
        failing_input.current_feedback_.q_current_ = Amperes{0.0F};
        failing_input.electrical_angle_ = ElectricalRad{0.0F};
        failing_input.v_bus_ = Volts{24.0F};

        CurrentLoopOutput failed_output{};
        failed_output.duty_.a_duty_ = 11.0F;
        failed_output.duty_.b_duty_ = 22.0F;
        failed_output.duty_.c_duty_ = 33.0F;
        failed_output.is_voltage_limited_ = true;
        ROBOT_FOC_TEST_EXPECT(
            loop.calculate(failing_input, failed_output) == ErrorCode::ErrorInvalidParam,
            "current_loop second-axis integral overflow rejected"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(failed_output.duty_.a_duty_, 11.0F)
                && nearly_equal(failed_output.duty_.b_duty_, 22.0F)
                && nearly_equal(failed_output.duty_.c_duty_, 33.0F)
                && failed_output.is_voltage_limited_,
            "current_loop failed transaction preserves duty output"
        );

        CurrentLoopInput zero_input{};
        zero_input.electrical_angle_ = ElectricalRad{0.0F};
        zero_input.v_bus_ = Volts{24.0F};
        CurrentLoopOutput output_after_failure{};
        CurrentLoopOutput baseline_output{};
        ROBOT_FOC_TEST_ASSERT(
            loop.calculate(zero_input, output_after_failure) == ErrorCode::Ok,
            "current_loop calculate after failed transaction succeeds"
        );
        ROBOT_FOC_TEST_ASSERT(
            baseline.calculate(zero_input, baseline_output) == ErrorCode::Ok,
            "current_loop baseline zero-state calculate succeeds"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(output_after_failure.duty_.a_duty_, baseline_output.duty_.a_duty_)
                && nearly_equal(output_after_failure.duty_.b_duty_, baseline_output.duty_.b_duty_)
                && nearly_equal(output_after_failure.duty_.c_duty_, baseline_output.duty_.c_duty_)
                && (output_after_failure.is_voltage_limited_
                    == baseline_output.is_voltage_limited_),
            "current_loop second-axis failure leaves both pi states unchanged"
        );
    }
}
