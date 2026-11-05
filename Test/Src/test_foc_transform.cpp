/**
 * @file test_foctransform.cpp
 * @brief FOC Clarke / Park / InverseClarke / InversePark 坐标变换单元测试
 * @details
 * 测试对象：robot_foc::algorithm::FocTransform
 * 约定：等幅值Clarke变换；角度统一为电角度 ElectricalRad，单位 rad
 * 测试策略：黑盒测试，仅调用对外公开接口，不直接访问内部计算函数
 * 测试场景：
 *  1. 三相ABC输入 Clarke 变换
 *  2. 两相AB输入 Clarke 变换
 *  3. 逆Clarke αβ 还原三相ABC
 *  4. Park 特征角度变换（0、π/2、π）
 *  5. 逆Park 变换
 *  6. 完整变换链路闭环校验 ABC → Clarke → Park → InvPark → InvClarke
 *  7. 电流、电压两套重载接口分别验证
 * @note
 *  - 浮点数比较使用容忍阈值 kEps，避免浮点误差导致误报失败
 *  - 全部为无状态纯数学变换，无需复位
 */
#include <cstring>
#include <cstdio>

#include "ALGORITHM/foc_transform.hpp"
#include "test_helper.hpp"

void test_foc_transform()
{
    using robot_foc::algorithm::FocTransform;
    using robot_foc::common::ErrorCode;
    using robot_foc::common::ElectricalRad;

    // 三相输入Clarke变换
    {
        robot_foc::common::ABCCurrent ABCcurr{};
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr{};

        ABCcurr.a_current_ = 1.0F;
        ABCcurr.b_current_ = 0.0F;
        ABCcurr.c_current_ = -1.0F;

        auto ret = FocTransform::clarke(ABCcurr,AlphaBetacurr);
        auto eq_alpha = nearly_equal(AlphaBetacurr.alpha_current_, 1.0F);
        auto eq_beta = nearly_equal(AlphaBetacurr.beta_current_,1.0F/(float)std::sqrt(3));

        ROBOT_FOC_TEST_EXPECT((ret == ErrorCode::Ok) && eq_alpha && eq_beta,"clarke three phase ok");
    }

    // 两相输入Clarke变换
    {
        robot_foc::common::ABCurrent ABcurr{};
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr{};

        ABcurr.a_current_ = 1.0F;
        ABcurr.b_current_ = 0.0F;

        auto ret = FocTransform::clarke(ABcurr,AlphaBetacurr);

        auto eq_alpha = nearly_equal(AlphaBetacurr.alpha_current_, 1.0F);
        auto eq_beta  = nearly_equal(AlphaBetacurr.beta_current_, 1.0F / (float)std::sqrt(3));

        ROBOT_FOC_TEST_EXPECT((ret == ErrorCode::Ok) && eq_alpha && eq_beta,"clarke two phase ok");    }

    // Park变换
    {
        constexpr float PI = 3.14159265358979323846F;
        constexpr float HALF_PI = PI / 2.0F;
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr{};
        robot_foc::common::DQCurrent DQcurr{};
        ElectricalRad theta_1{0.0F};
        ElectricalRad theta_2{HALF_PI};
        ElectricalRad theta_3{PI};

        AlphaBetacurr.alpha_current_ = 1.0F;
        AlphaBetacurr.beta_current_ = 0.0F;

        auto ret_1 = FocTransform::park(AlphaBetacurr,theta_1,DQcurr);
        auto eq_d1 = nearly_equal(DQcurr.d_current_, 1.0F);
        auto eq_q1 = nearly_equal(DQcurr.q_current_, 0.0F);
        ROBOT_FOC_TEST_EXPECT((ret_1 == ErrorCode::Ok) && eq_d1 && eq_q1,"park 0 rad ok");

        auto ret_2 = FocTransform::park(AlphaBetacurr,theta_2,DQcurr);
        auto eq_d2 = nearly_equal(DQcurr.d_current_, 0.0F);
        auto eq_q2 = nearly_equal(DQcurr.q_current_, -1.0F);
        ROBOT_FOC_TEST_EXPECT((ret_2 == ErrorCode::Ok) && eq_d2 && eq_q2,"park pi/2 rad ok");

        auto ret_3 = FocTransform::park(AlphaBetacurr,theta_3,DQcurr);
        auto eq_d3 = nearly_equal(DQcurr.d_current_, -1.0F);
        auto eq_q3 = nearly_equal(DQcurr.q_current_, 0.0F);
        ROBOT_FOC_TEST_EXPECT((ret_3 == ErrorCode::Ok) && eq_d3 && eq_q3,"park pi rad ok");
    }

    // Park逆变换
    {
        robot_foc::common::DQCurrent DQcurr{};
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr{};
        ElectricalRad theta{0.0F};

        DQcurr.d_current_ = 1.0F;
        DQcurr.q_current_ = 0.0F;

        auto ret = FocTransform::inverse_park(DQcurr,theta,AlphaBetacurr);

        auto eq_alpha = nearly_equal(AlphaBetacurr.alpha_current_, 1.0F);
        auto eq_beta  = nearly_equal(AlphaBetacurr.beta_current_, 0.0F);

        ROBOT_FOC_TEST_EXPECT((ret == ErrorCode::Ok) && eq_alpha && eq_beta,"inverse park 0 rad ok");
    }

    // 完整变换链路闭环校验 ABC → Clarke → Park → InvPark → InvClarke
    {
        robot_foc::common::ABCCurrent ABCcurr_1{};
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr_1{};
        robot_foc::common::DQCurrent DQcurr{};
        robot_foc::common::AlphaBetaCurrent AlphaBetacurr_2{};
        robot_foc::common::ABCCurrent ABCcurr_2{};
        ElectricalRad theta{0.0F};

        ABCcurr_1.a_current_ = 1.0F;
        ABCcurr_1.b_current_ = 0.0F;
        ABCcurr_1.c_current_ = -1.0F;

        auto r_clarke = FocTransform::clarke(ABCcurr_1, AlphaBetacurr_1);
        ROBOT_FOC_TEST_EXPECT(r_clarke == ErrorCode::Ok, "clarke in chain ok");

        auto r_park = FocTransform::park(AlphaBetacurr_1, theta, DQcurr);
        ROBOT_FOC_TEST_EXPECT(r_park == ErrorCode::Ok, "park in chain ok");

        auto r_invpark = FocTransform::inverse_park(DQcurr, theta, AlphaBetacurr_2);
        ROBOT_FOC_TEST_EXPECT(r_invpark == ErrorCode::Ok, "inv park in chain ok");

        auto ret_1 = nearly_equal(AlphaBetacurr_2.alpha_current_, AlphaBetacurr_1.alpha_current_);
        auto ret_2 = nearly_equal(AlphaBetacurr_2.beta_current_, AlphaBetacurr_1.beta_current_);
        ROBOT_FOC_TEST_EXPECT(ret_1 && ret_2, "alpha beta restore ok");

        std::cout << "alpha1:" << AlphaBetacurr_1.alpha_current_ << " alpha2:" << AlphaBetacurr_2.alpha_current_ << "\n";
        std::cout << "beta1:" << AlphaBetacurr_1.beta_current_ << " beta2:" << AlphaBetacurr_2.beta_current_ << "\n";

        auto r_invclarke = FocTransform::inverse_clarke(AlphaBetacurr_2, ABCcurr_2);
        ROBOT_FOC_TEST_EXPECT(r_invclarke == ErrorCode::Ok, "inv clarke in chain ok");

        auto ret_4 = nearly_equal(ABCcurr_2.a_current_, ABCcurr_1.a_current_);
        auto ret_5 = nearly_equal(ABCcurr_2.b_current_, ABCcurr_1.b_current_);
        auto ret_6 = nearly_equal(ABCcurr_2.c_current_, ABCcurr_1.c_current_);

        std::cout << ABCcurr_2.a_current_ << " " << ret_4 << "\n";
        std::cout << ABCcurr_2.b_current_ << " " << ret_5 << "\n";
        std::cout << ABCcurr_2.c_current_ << " " << ret_6 << "\n";

        ROBOT_FOC_TEST_EXPECT(ret_4 && ret_5 && ret_6, "full chain restore ABC ok");
    }

    // 电压重载测试
    {
        robot_foc::common::ABCVoltage ABCvolt{};
        robot_foc::common::AlphaBetaVoltage AlphaBetavolt{};

        ABCvolt.a_volt_ = 1.0F;
        ABCvolt.b_volt_ = 0.0F;
        ABCvolt.c_volt_ = -1.0F;

        auto ret = FocTransform::clarke(ABCvolt, AlphaBetavolt);

        auto eq_alpha = nearly_equal(AlphaBetavolt.alpha_volt_, 1.0F);
        auto eq_beta  = nearly_equal(AlphaBetavolt.beta_volt_, 1.0F / (float)std::sqrt(3));

        ROBOT_FOC_TEST_EXPECT(ret == ErrorCode::Ok && eq_alpha && eq_beta, "voltage clarke ok");
    }
}
