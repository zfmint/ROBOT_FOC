/**
 * @file test_svpwm.cpp
 * @brief Svpwm七段式SVPWM算法单元测试
 * @details
 * 黑盒测试SVPWM对外静态接口svpwm_calculate；
 * ⚠️电压限幅已迁移至独立模块VoltageLimit，调用前必须先做限幅；
 * 覆盖零矢量、6扇区代表点；非法参数；未限幅超限输入负用例；
 * 校验duty范围约束、失败时输出缓冲区不被篡改。
 */
#include <cmath>
#include "ALGORITHM/svpwm.hpp"
#include "ALGORITHM/voltage_limit.hpp"
#include "test_helper.hpp"
#include "error_code.hpp"

void test_svpwm()
{
    using namespace robot_foc::algorithm;
    using namespace robot_foc::common;

    // -------- 测试1：零矢量输入 --------
    {
        AlphaBetaVoltage vec_in{};
        VoltageLimitOut limit_res{};
        SvpwmDuty duty_out{};
        const Volts vbus{24.0F};

        auto ret_limit = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
        ROBOT_FOC_TEST_EXPECT(ret_limit == ErrorCode::Ok, "svpwm zero-vector pre-limit ok");

        auto ret_calc = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::Ok, "svpwm zero-vector calculate return Ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty, 0.5F), "svpwm zero duty A = 0.5");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.b_duty, 0.5F), "svpwm zero duty B = 0.5");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.c_duty, 0.5F), "svpwm zero duty C = 0.5");
    }

    // -------- 测试2：线性区 α=5V β=0，先限幅再送入svpwm --------
    {
        AlphaBetaVoltage vec_in{};
        VoltageLimitOut limit_res{};
        SvpwmDuty duty_out{};
        const Volts vbus{24.0F};
        vec_in.alpha_volt = Volts{5.0F};
        vec_in.beta_volt  = Volts{0.0F};

        auto ret_limit = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
        ROBOT_FOC_TEST_EXPECT(ret_limit == ErrorCode::Ok, "svpwm linear pre-limit ok");

        auto ret_calc = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::Ok, "svpwm linear region calculate ok");
        ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "linear dutyA in [0,1]");
        ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "linear dutyB in [0,1]");
        ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "linear dutyC in [0,1]");
    }

    // -------- 测试3：【负用例】未限幅超限矢量直接喂给svpwm_calculate，预期ErrorInvalidParam --------
    {
        AlphaBetaVoltage vec_raw{};
        SvpwmDuty duty_out{};
        // 预先填充脏值，校验失败不篡改输出
        duty_out.a_duty = 99.0F;
        duty_out.b_duty = 99.0F;
        duty_out.c_duty = 99.0F;
        const Volts vbus{24.0F};
        // α=20V，远超六边形α轴上限16V，未经过VoltageLimit
        vec_raw.alpha_volt = Volts{20.0F};
        vec_raw.beta_volt  = Volts{0.0F};

        auto ret_calc = Svpwm::svpwm_calculate(vec_raw, vbus, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::ErrorInvalidParam, "svpwm input over hex without limit return invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty, 99.0F), "svpwm fail preserve dutyA");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.b_duty, 99.0F), "svpwm fail preserve dutyB");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.c_duty, 99.0F), "svpwm fail preserve dutyC");
    }

    // -------- 测试4：非法母线电压Vbus=0，输出不篡改 --------
    {
        AlphaBetaVoltage vec_in{};
        SvpwmDuty duty_out{};
        duty_out.a_duty = 88.0F;
        duty_out.b_duty = 88.0F;
        duty_out.c_duty = 88.0F;
        vec_in.alpha_volt = Volts{5.0F};
        vec_in.beta_volt  = Volts{0.0F};

        auto ret = Svpwm::svpwm_calculate(vec_in, Volts{0.0F}, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "svpwm vbus zero invalid param");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty, 88.0F), "svpwm vbus zero preserve dutyA");
    }

    // -------- 测试5：非法浮点数NAN输入 --------
    {
        AlphaBetaVoltage vec_in{};
        SvpwmDuty duty_out{};
        duty_out.a_duty = 77.0F;
        const Volts vbus{24.0F};
        vec_in.alpha_volt = Volts{NAN};

        auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "svpwm NAN input invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty,77.0F), "svpwm NAN preserve output");
    }

    // -------- 测试6：六个扇区代表点，先限幅再计算，校验duty全部落在[0,1] --------
    {
        const Volts vbus{24.0F};
        const float sector_points[6][2] = {
            { 8.0F,  0.0F},     // S1
            { 4.0F,  7.0F},     // S2
            {-4.0F,  7.0F},     // S3
            {-8.0F,  0.0F},     // S4
            {-4.0F, -7.0F},     // S5
            { 4.0F, -7.0F}      // S6
        };

        // S1
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[0][0]};
            vec_in.beta_volt  = Volts{sector_points[0][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector1 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector1 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector1 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector1 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector1 dutyC in [0,1]");
        }
        // S2
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[1][0]};
            vec_in.beta_volt  = Volts{sector_points[1][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector2 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector2 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector2 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector2 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector2 dutyC in [0,1]");
        }
        // S3
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[2][0]};
            vec_in.beta_volt  = Volts{sector_points[2][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector3 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector3 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector3 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector3 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector3 dutyC in [0,1]");
        }
        // S4
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[3][0]};
            vec_in.beta_volt  = Volts{sector_points[3][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector4 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector4 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector4 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector4 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector4 dutyC in [0,1]");
        }
        // S5
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[4][0]};
            vec_in.beta_volt  = Volts{sector_points[4][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector5 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector5 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector5 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector5 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector5 dutyC in [0,1]");
        }
        // S6
        {
            AlphaBetaVoltage vec_in{};
            VoltageLimitOut limit_res{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt = Volts{sector_points[5][0]};
            vec_in.beta_volt  = Volts{sector_points[5][1]};
            auto ret_lim = VoltageLimit::hexagon_limit(vec_in, vbus, limit_res);
            ROBOT_FOC_TEST_EXPECT(ret_lim == ErrorCode::Ok, "Sector6 pre-limit ok");
            auto ret = Svpwm::svpwm_calculate(limit_res.alphabeta_out, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector6 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty >= -kTolerance) && (duty_out.a_duty <= 1.0F + kTolerance), "Sector6 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty >= -kTolerance) && (duty_out.b_duty <= 1.0F + kTolerance), "Sector6 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty >= -kTolerance) && (duty_out.c_duty <= 1.0F + kTolerance), "Sector6 dutyC in [0,1]");
        }
    }

    // -------- 测试7：不同母线电压 12V /48V --------
    {
        AlphaBetaVoltage vec_in{};
        VoltageLimitOut limit_res_12{};
        VoltageLimitOut limit_res_48{};
        SvpwmDuty duty12{};
        SvpwmDuty duty48{};
        vec_in.alpha_volt = Volts{5.0F};
        vec_in.beta_volt  = Volts{2.0F};

        auto ret_lim12 = VoltageLimit::hexagon_limit(vec_in, Volts{12.0F}, limit_res_12);
        auto ret_lim48 = VoltageLimit::hexagon_limit(vec_in, Volts{48.0F}, limit_res_48);
        ROBOT_FOC_TEST_EXPECT(ret_lim12 == ErrorCode::Ok, "vbus12 pre-limit ok");
        ROBOT_FOC_TEST_EXPECT(ret_lim48 == ErrorCode::Ok, "vbus48 pre-limit ok");

        auto ret12 = Svpwm::svpwm_calculate(limit_res_12.alphabeta_out, Volts{12.0F}, duty12);
        auto ret48 = Svpwm::svpwm_calculate(limit_res_48.alphabeta_out, Volts{48.0F}, duty48);
        ROBOT_FOC_TEST_EXPECT(ret12 == ErrorCode::Ok, "vbus=12V calculate ok");
        ROBOT_FOC_TEST_EXPECT(ret48 == ErrorCode::Ok, "vbus=48V calculate ok");
    }
}
