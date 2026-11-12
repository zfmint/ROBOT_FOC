/**
 * @file test_voltage_limit.cpp
 * @brief VoltageLimit电压限幅模块单元测试
 * @details
 * 黑盒测试六边形I区限幅 hexagon_limit、圆形内切圆限幅 circle_limit；
 * 覆盖零矢量、线性区、超限钳位、非法母线电压、NaN/Inf输入；
 * 校验错误返回时输出缓冲区不被篡改。
 */
#include "ALGORITHM/voltage_limit.hpp"

#include <cmath>

#include "test_helper.hpp"
#include "error_code.hpp"

void test_voltage_limit()
{
    using namespace robot_foc::algorithm;
    using namespace robot_foc::common;
    constexpr float SQRT3 = 1.73205080757F;

    // ========== hexagon_limit 六边形I区限幅测试 ==========
    // 测试1：六边形限幅 - 零矢量，vbus合法
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon zero vector ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_), 0.0F), "hexagon zero alpha is 0");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_), 0.0F), "hexagon zero beta is 0");
    }

    // 测试2：六边形限幅 - 零矢量 + 非法vbus(0、负数)，输出缓冲区不篡改
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        out.alpha_volt_ = Volts{88.0F};
        out.beta_volt_  = Volts{88.0F};

        auto ret_v0 = VoltageLimit::hexagon_limit(in, Volts{0.0F}, out);
        ROBOT_FOC_TEST_EXPECT(ret_v0 == ErrorCode::ErrorInvalidParam, "hexagon zero vector vbus=0 invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_), 88.0F), "hexagon fail preserve alpha");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_), 88.0F), "hexagon fail preserve beta");

        auto ret_neg = VoltageLimit::hexagon_limit(in, Volts{-12.0F}, out);
        ROBOT_FOC_TEST_EXPECT(ret_neg == ErrorCode::ErrorInvalidParam, "hexagon zero vector vbus neg invalid");
    }

    // 测试3：六边形限幅 - α轴超限，vbus=24V，α轴六边形上限 = 2/3*Vbus =16.0V
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        const float hex_max_alpha = (2.0F / 3.0F) * static_cast<float>(vbus);

        in.alpha_volt_ = Volts{20.0F};
        in.beta_volt_  = Volts{0.0F};
        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon alpha overmod ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_), hex_max_alpha), "hexagon alpha clamped");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_), 0.0F), "hexagon beta remain zero");
    }

    // 测试3-1：六边形顶点方向(0°)超限，验证角度保持不变
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        // 输入：0°顶点方向，幅值20V，超出六边形边界
        const float angle_in_rad = 0.0F;
        const float amp_in = 20.0F;
        in.alpha_volt_ = Volts{amp_in * std::cosf(angle_in_rad)};
        in.beta_volt_  = Volts{amp_in * std::sinf(angle_in_rad)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon vertex 0deg over-limit ok");

        float a_out = static_cast<float>(out.alpha_volt_);
        float b_out = static_cast<float>(out.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);

        // 0°顶点方向的β始终为0，径向缩放后角度保持不变
        ROBOT_FOC_TEST_EXPECT(nearly_equal(angle_out, angle_in_rad), "hexagon vertex 0deg direction preserved");
    }

    // 测试3-2：六边形边中点方向(30°)超限，验证角度保持不变
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        const float edge_midpoint_angle_rad = 30.0F * 3.1415926F / 180.0F;
        const float amp_raw = 20.0F; // 大于六边形边中点允许幅值

        in.alpha_volt_ = Volts{amp_raw * std::cosf(edge_midpoint_angle_rad)};
        in.beta_volt_  = Volts{amp_raw * std::sinf(edge_midpoint_angle_rad)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon edge-midpoint 30deg over-limit ok");

        float a_out = static_cast<float>(out.alpha_volt_);
        float b_out = static_cast<float>(out.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);

        // 边中点方向径向限幅，原始角度必须保留
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(angle_out, edge_midpoint_angle_rad),
            "hexagon edge-midpoint keep input angle");
    }

    // 测试3-3：第一扇区15°方向超限，验证径向缩放保持角度
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        const float angle_in = 15.0F * 3.1415926F / 180.0F;
        const float amp_in = 20.0F;
        in.alpha_volt_ = Volts{amp_in * std::cosf(angle_in)};
        in.beta_volt_  = Volts{amp_in * std::sinf(angle_in)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon 15deg over-limit ok");

        float a_out = static_cast<float>(out.alpha_volt_);
        float b_out = static_cast<float>(out.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);
        float amp_out = std::sqrtf(a_out*a_out + b_out*b_out);

        // 本实现径向缩放：角度必须和输入保持一致
        ROBOT_FOC_TEST_EXPECT(nearly_equal(angle_out, angle_in), "hexagon 15deg keep input angle");
        // 幅值被压缩，比原始输入小
        ROBOT_FOC_TEST_EXPECT(amp_out < amp_in - 1e-3F, "hexagon 15deg amplitude clamped down");
    }

    // 测试3-4：六个扇区中心方向超限，验证幅值钳位和方向保持
    {
        const Volts vbus{24.0F};
        const float edge_midpoint_amp = static_cast<float>(vbus) / SQRT3;
        const float half_raw_amp = 10.0F;
        const float sector_center[6][2] = {
            { half_raw_amp * SQRT3,  half_raw_amp},
            { 0.0F,                  2.0F * half_raw_amp},
            {-half_raw_amp * SQRT3,  half_raw_amp},
            {-half_raw_amp * SQRT3, -half_raw_amp},
            { 0.0F,                 -2.0F * half_raw_amp},
            { half_raw_amp * SQRT3, -half_raw_amp}
        };

        for (const auto& point : sector_center)
        {
            AlphaBetaVoltage in{};
            AlphaBetaVoltage out{};
            in.alpha_volt_ = Volts{point[0]};
            in.beta_volt_ = Volts{point[1]};

            const auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
            const float alpha_out = static_cast<float>(out.alpha_volt_);
            const float beta_out = static_cast<float>(out.beta_volt_);
            const float output_amp = std::hypot(alpha_out, beta_out);
            const float cross = point[0] * beta_out - point[1] * alpha_out;
            const float dot = point[0] * alpha_out + point[1] * beta_out;

            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon sector-center limit ok");
            ROBOT_FOC_TEST_EXPECT(
                nearly_equal(output_amp, edge_midpoint_amp, 1.0e-3F),
                "hexagon sector-center amplitude clamped");
            ROBOT_FOC_TEST_EXPECT(
                nearly_equal(cross, 0.0F, 1.0e-3F) && (dot > 0.0F),
                "hexagon sector-center direction preserved");
        }
    }

    // 测试3-5：六个60°边界方向超限，验证六边形顶点幅值
    {
        const Volts vbus{24.0F};
        const float vertex_amp = (2.0F / 3.0F) * static_cast<float>(vbus);
        const float half_raw_amp = 10.0F;
        const float sector_boundary[6][2] = {
            { 2.0F * half_raw_amp,  0.0F},
            { half_raw_amp,          half_raw_amp * SQRT3},
            {-half_raw_amp,          half_raw_amp * SQRT3},
            {-2.0F * half_raw_amp,  0.0F},
            {-half_raw_amp,         -half_raw_amp * SQRT3},
            { half_raw_amp,         -half_raw_amp * SQRT3}
        };

        for (const auto& point : sector_boundary)
        {
            AlphaBetaVoltage in{};
            AlphaBetaVoltage out{};
            in.alpha_volt_ = Volts{point[0]};
            in.beta_volt_ = Volts{point[1]};

            const auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
            const float alpha_out = static_cast<float>(out.alpha_volt_);
            const float beta_out = static_cast<float>(out.beta_volt_);
            const float output_amp = std::hypot(alpha_out, beta_out);
            const float cross = point[0] * beta_out - point[1] * alpha_out;
            const float dot = point[0] * alpha_out + point[1] * beta_out;

            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon sector-boundary limit ok");
            ROBOT_FOC_TEST_EXPECT(
                nearly_equal(output_amp, vertex_amp, 1.0e-3F),
                "hexagon sector-boundary amplitude clamped");
            ROBOT_FOC_TEST_EXPECT(
                nearly_equal(cross, 0.0F, 1.0e-3F) && (dot > 0.0F),
                "hexagon sector-boundary direction preserved");
        }
    }

    // 测试4：六边形限幅 - 线性区内，矢量不缩放
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        in.alpha_volt_ = Volts{5.0F};
        in.beta_volt_  = Volts{2.0F};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon linear region ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_), 5.0F), "hexagon linear alpha keep");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_), 2.0F), "hexagon linear beta keep");
    }

    // 测试5：六边形限幅 - NAN / INF非法输入，输出保持不变
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        out.alpha_volt_ = Volts{77.0F};
        out.beta_volt_  = Volts{77.0F};
        const Volts vbus{24.0F};

        in.alpha_volt_ = Volts{NAN};
        auto ret_nan = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "hexagon NAN input invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_),77.0F), "hexagon NAN preserve output");

        in.alpha_volt_ = Volts{INFINITY};
        auto ret_inf = VoltageLimit::hexagon_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret_inf == ErrorCode::ErrorInvalidParam, "hexagon INF input invalid");
    }

    // ========== circle_limit 圆形内切圆限幅测试 ==========
    // 测试6：圆形限幅 - 零矢量
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};

        auto ret = VoltageLimit::circle_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle zero vector ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_),0.0F), "circle zero alpha 0");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_),0.0F), "circle zero beta 0");
    }

    // 测试7：圆形限幅 - 超限径向缩放，保持角度；最大幅值 Vbus/sqrt(3)
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        const float circle_max_amp = static_cast<float>(vbus) / SQRT3; //≈13.856V

        // 30度方向，超出圆形边界，缩放，角度不变
        const float angle_30_rad = 30.0F * 3.1415926F / 180.0F;
        const float raw_amp = 20.0F;
        in.alpha_volt_ = Volts{raw_amp * std::cosf(angle_30_rad)};
        in.beta_volt_  = Volts{raw_amp * std::sinf(angle_30_rad)};

        auto ret = VoltageLimit::circle_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle over-amplitude ok");

        // 限幅后幅值等于圆形上限
        float out_a = static_cast<float>(out.alpha_volt_);
        float out_b = static_cast<float>(out.beta_volt_);
        float out_amp = std::sqrtf(out_a*out_a + out_b*out_b);
        ROBOT_FOC_TEST_EXPECT(nearly_equal(out_amp, circle_max_amp), "circle output amplitude clamped");

        // 校验角度不变 atan2(beta,alpha)
        float out_angle = std::atan2f(out_b, out_a);
        ROBOT_FOC_TEST_EXPECT(nearly_equal(out_angle, angle_30_rad), "circle keep vector angle");
    }

    // 测试8：圆形限幅 - 零矢量搭配非法vbus
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        out.alpha_volt_ = Volts{66.0F};
        out.beta_volt_  = Volts{66.0F};

        auto ret = VoltageLimit::circle_limit(in, Volts{0.0F}, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "circle zero vbus=0 invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.alpha_volt_),66.0F), "circle fail preserve alpha");
    }

    // 测试9：圆形限幅 - NAN输入
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        out.alpha_volt_ = Volts{55.0F};
        out.beta_volt_  = Volts{55.0F};
        const Volts vbus{24.0F};
        in.beta_volt_ = Volts{NAN};

        auto ret = VoltageLimit::circle_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "circle NAN input invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out.beta_volt_),55.0F), "circle NAN preserve output");
    }

    // 测试10：圆形限幅 - 大有限输入不因中间平方溢出而丢失方向
    {
        AlphaBetaVoltage in{};
        AlphaBetaVoltage out{};
        const Volts vbus{24.0F};
        const float large_finite = 1.0e20F;
        const float expected_component = static_cast<float>(vbus) / std::sqrt(6.0F);
        in.alpha_volt_ = Volts{large_finite};
        in.beta_volt_ = Volts{large_finite};

        const auto ret = VoltageLimit::circle_limit(in, vbus, out);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle large finite input ok");
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(static_cast<float>(out.alpha_volt_), expected_component, 1.0e-3F),
            "circle large finite alpha clamped");
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(static_cast<float>(out.beta_volt_), expected_component, 1.0e-3F),
            "circle large finite beta clamped");
    }
}
