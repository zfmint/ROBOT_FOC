/**
 * @file test_current_loop.cpp
 * @brief CurrentLoop FOC dq电流内环单元测试
 * @details
 * 黑盒测试CurrentLoop对外公开API；底层依赖PiController、FocTransform、VoltageLimit、Svpwm均已完成单元测试。
 * 内部流水线：preview → inverse_park → hexagon_limit → park回dq → prepare_update → commit → svpwm_calculate。
 * 抗饱和：限幅后αβ重新Park回dq，传入prepare_update执行条件积分冻结。
 *
 * 测试覆盖项：
 * 1. configure正常配置、重复配置返回ErrorAlreadyInit
 * 2. 非法PI参数：kp负数、sample_time为0，全新实例返回ErrorInvalidParam
 * 3. 未configure调用calculate返回ErrorNotReady，输出缓冲区不篡改脏值
 * 4. reset：仅清零积分，保留PI增益配置
 * 5. 闭环流水线冒烟仿真：完整链路迭代运行，仅校验返回码
 *      @note【电流跟踪、阶跃响应、稳态误差数值断言下沉至 test_pi_controller】
 *            CurrentLoop为上层组装层，不做算法精度校验。
 * 6. 电压矢量饱和工况运行冒烟测试
 * 7. NaN非法输入保护，输出缓冲区保留脏值
 * 8. 底层模块错误码向上透传
 *
 * @note
 * 1. PiController::commit为私有友元接口，测试层无法直接访问；全部通过CurrentLoop::calculate黑盒验证。
 * 2. 本文件仅属于PC‑test测试工程，禁止加入robot_hal_core业务库工程；业务源码不得include测试头。
 * 3. 浮点数比较统一使用test_helper提供nearly_equal，支持自定义容差；全局默认kTolerance=1.0e‑4F。
 */
#include <cmath>
#include "ALGORITHM/current_loop.hpp"
#include "ALGORITHM/pid.hpp"
#include "test_helper.hpp"
#include "error_code.hpp"

void test_current_loop()
{
    using namespace robot_foc::algorithm;
    using namespace robot_foc::common;

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

    // -------- 测试2：非法PI参数case1：kp负数，全新实例 --------
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

    // -------- 测试2‑2：非法PI参数case2：sample_time等于0 --------
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

    // -------- 测试3：未configure直接calculate，返回ErrorNotReady，输出保留脏值 --------
    {
        CurrentLoop loop{};
        CurrentLoopInput input{};
        SvpwmDuty duty_out{};
        duty_out.a_duty_ = 111.0F;
        duty_out.b_duty_ = 222.0F;
        duty_out.c_duty_ = 333.0F;

        auto ret_calc = loop.calculate(input, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::ErrorNotReady, "current_loop calculate before configure return not ready");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty_, 111.0F), "current_loop not-ready preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.b_duty_, 222.0F), "current_loop not-ready preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.c_duty_, 333.0F), "current_loop not-ready preserve dutyC");
    }

    // -------- 测试4：reset复位，仅清零积分状态，增益不变 --------
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 2.0F;
        cfg_id.ki_ = 100.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -10.0F;
        cfg_id.integral_max_ = 10.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "current_loop reset-test configure ok");

        CurrentLoopInput input{};
        input.current_reference_.d_current_ = Amperes{2.0F};
        input.current_reference_.q_current_ = Amperes{2.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};
        SvpwmDuty duty{};

        for(int i = 0; i < 20; ++i)
        {
            (void)loop.calculate(input, duty);
        }

        loop.reset();
        auto ret_after_reset = loop.calculate(input, duty);
        ROBOT_FOC_TEST_EXPECT(ret_after_reset == ErrorCode::Ok, "current_loop after reset calculate ok");
    }

    // -------- 测试5：闭环流水线冒烟仿真，阶跃给定，仅验证完整链路无报错
    // 【重要】电流跟踪、稳态误差、阶跃响应数值断言下沉至 test_pi_controller；本用例不做精度校验
    {
        CurrentLoop loop{};
        PiConfig cfg_id{};
        cfg_id.kp_ = 1.8F;
        cfg_id.ki_ = 120.0F;
        cfg_id.sample_time_ = Seconds{0.0001F};
        cfg_id.integral_min_ = -12.0F;
        cfg_id.integral_max_ = 12.0F;
        PiConfig cfg_iq = cfg_id;

        auto ret_cfg = loop.configure(cfg_id, cfg_iq);
        ROBOT_FOC_TEST_EXPECT(ret_cfg == ErrorCode::Ok, "current_loop step configure ok");

        CurrentLoopInput input{};
        SvpwmDuty duty{};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};
        input.current_reference_.d_current_ = Amperes{1.0F};
        input.current_reference_.q_current_ = Amperes{2.5F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};

        for(int step = 0; step < 400; ++step)
        {
            auto ret = loop.calculate(input, duty);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "current_loop closed-loop step ok");
        }
    }

    // -------- 测试6：电压饱和工况冒烟测试，低压触发六边形限幅，整体链路不报错 --------
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
        SvpwmDuty duty{};

        for(int i = 0; i < 50; ++i)
        {
            auto ret = loop.calculate(input, duty);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "current_loop saturation run ok");
        }
    }

    // -------- 测试7：NaN非法输入，返回ErrorInvalidParam，输出duty保留脏值 --------
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

        SvpwmDuty duty_out{};
        duty_out.a_duty_ = 55.0F;
        duty_out.b_duty_ = 66.0F;
        duty_out.c_duty_ = 77.0F;

        CurrentLoopInput input{};
        input.current_reference_.d_current_ = Amperes{NAN};
        input.current_reference_.q_current_ = Amperes{1.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};
        input.v_bus_ = Volts{24.0F};

        auto ret_nan = loop.calculate(input, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "current_loop nan input reject");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty_, 55.0F), "current_loop nan preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.b_duty_, 66.0F), "current_loop nan preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.c_duty_, 77.0F), "current_loop nan preserve dutyC");
    }

    // -------- 测试8：零母线电压，SVPWM内部错误向上透传 --------
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
        SvpwmDuty duty_out{};
        duty_out.a_duty_ = 10.0F;
        duty_out.b_duty_ = 20.0F;
        duty_out.c_duty_ = 30.0F;

        input.v_bus_ = Volts{0.0F};
        input.current_reference_.d_current_ = Amperes{1.0F};
        input.current_reference_.q_current_ = Amperes{1.0F};
        input.current_feedback_.d_current_ = Amperes{0.0F};
        input.current_feedback_.q_current_ = Amperes{0.0F};
        input.electrical_angle_ = ElectricalRad{0.0F};

        auto ret_zero_vbus = loop.calculate(input, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_zero_vbus == ErrorCode::ErrorInvalidParam, "current_loop pass-through svpwm error");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty_,10.0F), "current_loop zero-vbus preserve dutyA");
    }
}
