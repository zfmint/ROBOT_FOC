/**
 * @file test_voltage_limit.cpp
 * @brief VoltageLimit电压限幅模块单元测试
 * @details
 * 黑盒测试六边形I区限幅 hexagon_limit、圆形内切圆限幅 circle_limit；
 * 覆盖：零矢量、线性区(不限幅)、多方向超限钳位、非法母线电压、NaN/Inf、极大有限值输入；
 * 校验点：返回码、输出矢量、is_limited标志位、错误路径输出缓冲区不篡改；
 * 算法特性：两种限幅均为径向等比例缩放，输入输出矢量角度严格保持不变。
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
    // 测试1：六边形限幅 - 零矢量，合法母线，不触发限幅
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon zero vector ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), 0.0F), "hexagon zero alpha is 0");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), 0.0F), "hexagon zero beta is 0");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == false, "hexagon zero-vector no clamp");
    }

    // 测试2：六边形限幅 - 零矢量 + 非法vbus(0、负数)，输出缓冲区不篡改
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        // 预先填充脏值
        out_res.alphabeta_out_.alpha_volt_ = Volts{88.0F};
        out_res.alphabeta_out_.beta_volt_  = Volts{88.0F};
        out_res.is_limited_ = true;

        auto ret_v0 = VoltageLimit::hexagon_limit(in, Volts{0.0F}, out_res);
        ROBOT_FOC_TEST_EXPECT(ret_v0 == ErrorCode::ErrorInvalidParam, "hexagon zero vector vbus=0 invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), 88.0F), "hexagon fail preserve alpha");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), 88.0F), "hexagon fail preserve beta");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon fail preserve is_limited flag");

        auto ret_neg = VoltageLimit::hexagon_limit(in, Volts{-12.0F}, out_res);
        ROBOT_FOC_TEST_EXPECT(ret_neg == ErrorCode::ErrorInvalidParam, "hexagon zero vector vbus neg invalid");
    }

    // 测试3：六边形限幅 - α轴方向超限，校验幅值钳位、is_limited置true，角度不变
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float hex_max_alpha = (2.0F / 3.0F) * static_cast<float>(vbus);

        in.alpha_volt_ = Volts{20.0F};
        in.beta_volt_  = Volts{0.0F};
        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon alpha overmod ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), hex_max_alpha), "hexagon alpha clamped");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), 0.0F), "hexagon beta remain zero");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon alpha over set is_limited true");
    }

    // 测试3-1：六边形0°方向超限，径向缩放，角度保持不变
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float angle_in_rad = 0.0F;
        const float amp_in = 20.0F;
        in.alpha_volt_ = Volts{amp_in * std::cosf(angle_in_rad)};
        in.beta_volt_  = Volts{amp_in * std::sinf(angle_in_rad)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon vertex 0deg over-limit ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon 0deg over-limit flag true");

        float a_out = static_cast<float>(out_res.alphabeta_out_.alpha_volt_);
        float b_out = static_cast<float>(out_res.alphabeta_out_.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);
        ROBOT_FOC_TEST_EXPECT(nearly_equal(angle_out, angle_in_rad), "hexagon vertex 0deg direction preserved");
    }

    // 测试3-2：六边形30°方向超限，径向缩放，角度保持不变
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float edge_midpoint_angle_rad = 30.0F * 3.1415926F / 180.0F;
        const float amp_raw = 20.0F;
        in.alpha_volt_ = Volts{amp_raw * std::cosf(edge_midpoint_angle_rad)};
        in.beta_volt_  = Volts{amp_raw * std::sinf(edge_midpoint_angle_rad)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon edge-midpoint 30deg over-limit ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon 30deg over-limit flag true");

        float a_out = static_cast<float>(out_res.alphabeta_out_.alpha_volt_);
        float b_out = static_cast<float>(out_res.alphabeta_out_.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);
        ROBOT_FOC_TEST_EXPECT(nearly_equal(angle_out, edge_midpoint_angle_rad), "hexagon edge-midpoint keep input angle");
    }

    // 测试3-3：15°任意方向超限，径向缩放，角度不变，幅值压缩
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float angle_in = 15.0F * 3.1415926F / 180.0F;
        const float amp_in = 20.0F;
        in.alpha_volt_ = Volts{amp_in * std::cosf(angle_in)};
        in.beta_volt_  = Volts{amp_in * std::sinf(angle_in)};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon 15deg over-limit ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon 15deg over-limit flag true");

        float a_out = static_cast<float>(out_res.alphabeta_out_.alpha_volt_);
        float b_out = static_cast<float>(out_res.alphabeta_out_.beta_volt_);
        float angle_out = std::atan2f(b_out, a_out);
        float amp_out = std::hypot(a_out, b_out);

        ROBOT_FOC_TEST_EXPECT(nearly_equal(angle_out, angle_in), "hexagon 15deg keep input angle");
        ROBOT_FOC_TEST_EXPECT(amp_out < amp_in - 1e-3F, "hexagon 15deg amplitude clamped down");
    }

    // 测试3-4：6个扇区中心方向超限，校验方向不变、is_limited=true
    {
        const Volts vbus{24.0F};
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
            VoltageLimitOut out_res{};
            in.alpha_volt_ = Volts{point[0]};
            in.beta_volt_ = Volts{point[1]};

            const auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
            const float alpha_out = static_cast<float>(out_res.alphabeta_out_.alpha_volt_);
            const float beta_out = static_cast<float>(out_res.alphabeta_out_.beta_volt_);
            const float cross = point[0] * beta_out - point[1] * alpha_out;
            const float dot = point[0] * alpha_out + point[1] * beta_out;

            ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon sector-center limit ok");
            ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon sector-center is_limited true");
            ROBOT_FOC_TEST_EXPECT(nearly_equal(cross, 0.0F, 1.0e-3F) && (dot > 0.0F), "hexagon sector-center direction preserved");
        }
    }

    // 测试4：六边形限幅-线性区内矢量，不缩放，is_limited=false
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        in.alpha_volt_ = Volts{5.0F};
        in.beta_volt_  = Volts{2.0F};

        auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon linear region ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), 5.0F), "hexagon linear alpha keep");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), 2.0F), "hexagon linear beta keep");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == false, "hexagon linear no clamp flag");
    }

    // 测试5：六边形限幅-NAN / INF非法输入，输出缓冲区不篡改
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        out_res.alphabeta_out_.alpha_volt_ = Volts{77.0F};
        out_res.alphabeta_out_.beta_volt_  = Volts{77.0F};
        out_res.is_limited_ = true;
        const Volts vbus{24.0F};

        in.alpha_volt_ = Volts{NAN};
        auto ret_nan = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret_nan == ErrorCode::ErrorInvalidParam, "hexagon NAN input invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_),77.0F), "hexagon NAN preserve output");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon NAN preserve flag");

        in.alpha_volt_ = Volts{INFINITY};
        auto ret_inf = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret_inf == ErrorCode::ErrorInvalidParam, "hexagon INF input invalid");
    }

    // ========== circle_limit 圆形内切圆限幅测试 ==========
    // 测试6：圆形限幅-零矢量，不限幅
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};

        auto ret = VoltageLimit::circle_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle zero vector ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_),0.0F), "circle zero alpha 0");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_),0.0F), "circle zero beta 0");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == false, "circle zero-vector no clamp");
    }

    // 测试7：圆形限幅-超限径向缩放，保持角度，is_limited=true
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float circle_max_amp = static_cast<float>(vbus) / SQRT3;
        const float angle_30_rad = 30.0F * 3.1415926F / 180.0F;
        const float raw_amp = 20.0F;
        in.alpha_volt_ = Volts{raw_amp * std::cosf(angle_30_rad)};
        in.beta_volt_  = Volts{raw_amp * std::sinf(angle_30_rad)};

        auto ret = VoltageLimit::circle_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle over-amplitude ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "circle over-amplitude flag true");

        float out_a = static_cast<float>(out_res.alphabeta_out_.alpha_volt_);
        float out_b = static_cast<float>(out_res.alphabeta_out_.beta_volt_);
        float out_amp = std::hypot(out_a, out_b);
        float out_angle = std::atan2f(out_b, out_a);

        ROBOT_FOC_TEST_EXPECT(nearly_equal(out_amp, circle_max_amp), "circle output amplitude clamped");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(out_angle, angle_30_rad), "circle keep vector angle");
    }

    // 测试8：圆形限幅-零矢量搭配非法vbus，输出不篡改
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        out_res.alphabeta_out_.alpha_volt_ = Volts{66.0F};
        out_res.alphabeta_out_.beta_volt_  = Volts{66.0F};
        out_res.is_limited_ = true;

        auto ret = VoltageLimit::circle_limit(in, Volts{0.0F}, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "circle zero vbus=0 invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_),66.0F), "circle fail preserve alpha");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "circle fail preserve flag");
    }

    // 测试9：圆形限幅-NAN输入，输出缓冲区保持脏值
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        out_res.alphabeta_out_.alpha_volt_ = Volts{55.0F};
        out_res.alphabeta_out_.beta_volt_  = Volts{55.0F};
        out_res.is_limited_ = true;
        const Volts vbus{24.0F};
        in.beta_volt_ = Volts{NAN};

        auto ret = VoltageLimit::circle_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::ErrorInvalidParam, "circle NAN input invalid");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_),55.0F), "circle NAN preserve output");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "circle NAN preserve flag");
    }

    // 测试10：圆形限幅-极大有限输入，方向不丢失
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float large_finite = 1.0e20F;
        const float expected_component = static_cast<float>(vbus) / std::sqrt(6.0F);
        in.alpha_volt_ = Volts{large_finite};
        in.beta_volt_ = Volts{large_finite};

        const auto ret = VoltageLimit::circle_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle large finite input ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "circle large finite is_limited true");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), expected_component, 1.0e-3F), "circle large finite alpha clamped");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), expected_component, 1.0e-3F), "circle large finite beta clamped");
    }

    // 测试11：六边形限幅-极大有限输入不发生投影溢出
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        const float large_finite = 1.0e38F;
        in.alpha_volt_ = Volts{large_finite};
        in.beta_volt_ = Volts{large_finite};

        const auto ret = VoltageLimit::hexagon_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "hexagon large finite input ok");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == true, "hexagon large finite is_limited true");
        ROBOT_FOC_TEST_EXPECT(
            std::isfinite(out_res.alphabeta_out_.alpha_volt_)
            && std::isfinite(out_res.alphabeta_out_.beta_volt_),
            "hexagon large finite output remains finite"
        );
        ROBOT_FOC_TEST_EXPECT(
            nearly_equal(
                out_res.alphabeta_out_.alpha_volt_,
                out_res.alphabeta_out_.beta_volt_
            ),
            "hexagon large finite direction preserved"
        );
    }

    // 测试12：圆形限幅-矢量在线性区内，is_limited=false
    {
        AlphaBetaVoltage in{};
        VoltageLimitOut out_res{};
        const Volts vbus{24.0F};
        in.alpha_volt_ = Volts{5.0F};
        in.beta_volt_  = Volts{2.0F};

        auto ret = VoltageLimit::circle_limit(in, vbus, out_res);
        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok, "circle linear region ok");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.alpha_volt_), 5.0F), "circle linear alpha keep");
        ROBOT_FOC_TEST_EXPECT(nearly_equal(static_cast<float>(out_res.alphabeta_out_.beta_volt_), 2.0F), "circle linear beta keep");
        ROBOT_FOC_TEST_EXPECT(out_res.is_limited_ == false, "circle linear no clamp flag");
    }
}
