/**
 * @brief αβ电压矢量六边形限幅（I区过调制）
 * @param[in]  alphabeta_in  原始αβ参考电压矢量
 * @param[in]  v_bus  直流母线电压
 * @param[out] alphabeta_out 限幅后αβ电压矢量
 * @retval common::ErrorCode::Ok 限幅完成，矢量在线性区或已钳位
 * @retval common::ErrorCode::ErrorInvalidParam 输入非法
 * @note
 *  本实现为六边形限幅；当矢量超出六边形边界会做缩放，**有可能改变矢量角度**；
 *  若需要保持矢量角度不变，应使用圆形内切圆限幅。
 */
#include <cmath>

#include "ALGORITHM/svpwm.hpp"
#include "test_helper.hpp"
#include "error_code.hpp"

namespace robot_foc::test
{

} // namespace robot_foc::test

void test_svpwm()
{
    using namespace robot_foc::algorithm;
    using namespace robot_foc::common;

    // -------- 测试1：零矢量正常，vbus合法 --------
    {
        AlphaBetaVoltage vec_in{};
        AlphaBetaVoltage vec_limited{};
        SvpwmDuty duty_out{};
        const Volts vbus{24.0F};

        auto ret_limit = Svpwm::voltage_limit(vec_in, vbus, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_limit == ErrorCode::Ok, "ZeroVector voltage_limit return Ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.alpha_volt_), 0.0F), "ZeroVector limited alpha = 0");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.beta_volt_), 0.0F), "ZeroVector limited beta = 0");

        auto ret_calc = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret_calc == ErrorCode::Ok, "ZeroVector svpwm_calculate return Ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty_, 0.5F), "ZeroVector duty A = 0.5");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.b_duty_, 0.5F), "ZeroVector duty B = 0.5");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.c_duty_, 0.5F), "ZeroVector duty C = 0.5");
    }

    // -------- P1漏洞用例：零矢量 + vbus非法（0、负数） --------
    {
        AlphaBetaVoltage vec_in{};
        AlphaBetaVoltage vec_limited{};
        // 预先填充非零，校验失败输出不被改写
        vec_limited.alpha_volt_ = Volts{99.0F};
        vec_limited.beta_volt_  = Volts{99.0F};

        auto ret_v0 = Svpwm::voltage_limit(vec_in, Volts{0.0F}, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_v0 == ErrorCode::ErrorInvalidParam, "ZeroVector + vbus=0 returns ErrorInvalidParam");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.alpha_volt_),99.0F), "failed case output buffer unchanged alpha");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.beta_volt_),99.0F), "failed case output buffer unchanged beta");

        auto ret_neg = Svpwm::voltage_limit(vec_in, Volts{-12.0F}, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_neg == ErrorCode::ErrorInvalidParam, "ZeroVector + negative vbus returns ErrorInvalidParam");
    }

    // -------- 测试2：线性区 α=5V β=0 --------
    {
        AlphaBetaVoltage vec_in{};
        AlphaBetaVoltage vec_limited{};
        const Volts vbus{24.0F};
        vec_in.alpha_volt_ = Volts{5.0F};
        vec_in.beta_volt_  = Volts{0.0F};

        auto ret_limit = Svpwm::voltage_limit(vec_in, vbus, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_limit == ErrorCode::Ok, "LinearRegion voltage_limit Ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.alpha_volt_),5.0F), "LinearRegion alpha keep input");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.beta_volt_),0.0F), "LinearRegion beta keep input");
    }

    // -------- 测试3：六边形限幅 α轴超限 20V，vbus=24V，预期16V --------
    {
        AlphaBetaVoltage vec_in{};
        AlphaBetaVoltage vec_limited{};
        const Volts vbus{24.0F};
        const float hex_max_alpha = (2.0F / 3.0F) * static_cast<float>(vbus);
        vec_in.alpha_volt_ = Volts{20.0F};
        vec_in.beta_volt_  = Volts{0.0F};

        auto ret_limit = Svpwm::voltage_limit(vec_in, vbus, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_limit == ErrorCode::Ok, "OverMod voltage_limit Ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.alpha_volt_), hex_max_alpha), "OverMod alpha clamped to hexagon boundary");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.beta_volt_), 0.0F), "OverMod beta remains zero");
    }

    // -------- 测试4：非法浮点数 NAN / INF，输出不修改 --------
    {
        AlphaBetaVoltage vec_in{};
        AlphaBetaVoltage vec_limited{};
        vec_limited.alpha_volt_ = Volts{77.0F};
        vec_limited.beta_volt_  = Volts{77.0F};
        const Volts vbus{24.0F};

        vec_in.alpha_volt_ = Volts{NAN};
        auto ret_nan = Svpwm::voltage_limit(vec_in, vbus, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "NaN input return ErrorInvalidParam");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(vec_limited.alpha_volt_),77.0F), "NaN‑fail output preserve alpha");

        vec_in.alpha_volt_ = Volts{INFINITY};
        auto ret_inf = Svpwm::voltage_limit(vec_in, vbus, vec_limited);
        ROBOT_FOC_TEST_EXPECT(ret_inf == ErrorCode::ErrorInvalidParam, "INF input return ErrorInvalidParam");
    }

    // -------- 测试5：6个扇区代表点，校验duty范围始终 [0,1] --------
    {
        const Volts vbus{24.0F};
        // 扇区代表点：{alpha,beta}
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
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[0][0]};
            vec_in.beta_volt_  = Volts{sector_points[0][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector1 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector1 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector1 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector1 dutyC in [0,1]");
        }
        // S2
        {
            AlphaBetaVoltage vec_in{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[1][0]};
            vec_in.beta_volt_  = Volts{sector_points[1][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector2 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector2 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector2 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector2 dutyC in [0,1]");
        }
        // S3
        {
            AlphaBetaVoltage vec_in{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[2][0]};
            vec_in.beta_volt_  = Volts{sector_points[2][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector3 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector3 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector3 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector3 dutyC in [0,1]");
        }
        // S4
        {
            AlphaBetaVoltage vec_in{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[3][0]};
            vec_in.beta_volt_  = Volts{sector_points[3][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector4 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector4 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector4 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector4 dutyC in [0,1]");
        }
        // S5
        {
            AlphaBetaVoltage vec_in{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[4][0]};
            vec_in.beta_volt_  = Volts{sector_points[4][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector5 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector5 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector5 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector5 dutyC in [0,1]");
        }
        // S6
        {
            AlphaBetaVoltage vec_in{};
            SvpwmDuty duty_out{};
            vec_in.alpha_volt_ = Volts{sector_points[5][0]};
            vec_in.beta_volt_  = Volts{sector_points[5][1]};
            auto ret = Svpwm::svpwm_calculate(vec_in, vbus, duty_out);
            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "Sector6 calculate ok");
            ROBOT_FOC_TEST_EXPECT((duty_out.a_duty_ >= -kTolerance) && (duty_out.a_duty_ <= 1.0F + kTolerance), "Sector6 dutyA in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.b_duty_ >= -kTolerance) && (duty_out.b_duty_ <= 1.0F + kTolerance), "Sector6 dutyB in [0,1]");
            ROBOT_FOC_TEST_EXPECT((duty_out.c_duty_ >= -kTolerance) && (duty_out.c_duty_ <= 1.0F + kTolerance), "Sector6 dutyC in [0,1]");
        }
    }

    // -------- 测试6：不同母线电压 12V / 48V --------
    {
        AlphaBetaVoltage vec_in{};
        SvpwmDuty duty12{};
        SvpwmDuty duty48{};
        vec_in.alpha_volt_ = Volts{5.0F};
        vec_in.beta_volt_  = Volts{2.0F};

        auto ret12 = Svpwm::svpwm_calculate(vec_in, Volts{12.0F}, duty12);
        auto ret48 = Svpwm::svpwm_calculate(vec_in, Volts{48.0F}, duty48);
        ROBOT_FOC_TEST_EXPECT(ret12 == ErrorCode::Ok, "vbus=12V ok");
        ROBOT_FOC_TEST_EXPECT(ret48 == ErrorCode::Ok, "vbus=48V ok");
    }

    // -------- 测试7：svpwm_calculate非法vbus，输出不篡改 --------
    {
        AlphaBetaVoltage vec_in{};
        SvpwmDuty duty_out{};
        duty_out.a_duty_ = 99.0F;
        duty_out.b_duty_ = 99.0F;
        duty_out.c_duty_ = 99.0F;
        vec_in.alpha_volt_ = Volts{5.0F};
        vec_in.beta_volt_  = Volts{0.0F};

        auto ret = Svpwm::svpwm_calculate(vec_in, Volts{0.0F}, duty_out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "svpwm_calc vbus zero invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(duty_out.a_duty_,99.0F), "svpwm calc fail preserve dutyA");
    }
}