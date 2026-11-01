/**
 * @file test_foctransform.cpp
 * @brief FOC Clarke / Park / InverseClarke / InversePark 坐标变换单元测试
 * @details
 * 测试对象：robot_foc::algorithm::FocTransform
 * 约定：等幅值Clarke变换；角度统一为电角度 ElectricalRad，单位 rad
 * 测试策略：黑盒测试，仅调用对外公开接口，不直接访问内部计算函数
 * 测试场景：
 *  1. 三相ABC输入 Clarke 变换
 *  2. 两相AB输入 Clarke 变换（自动推导C = -A-B）
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

    // 三相输入Clarke变换
    {
        robot_foc::algorithm::ABCCurrent ABCcurr{};
        robot_foc::algorithm::AlphaBetaCurrent AlphaBetacurr{};

        ABCcurr.a_current_ = 1.0F;
        ABCcurr.b_current_ = 0.0F;
        ABCcurr.c_current_ = -1.0F;

        auto ret = FocTransform::clarke(ABCcurr,AlphaBetacurr);

        std::cout<<AlphaBetacurr.alpha_current_<<"\n"<<AlphaBetacurr.beta_current_<<"\n";
        ROBOT_FOC_TEST_EXPECT((ret == ErrorCode::Ok),"clarke three phase ok");
    }
}
