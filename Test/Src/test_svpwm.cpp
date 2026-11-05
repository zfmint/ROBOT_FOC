/**
 * @brief SVPWM单元测试函数
 * @details 对SVPWM进行算法逻辑、状态机、边界行为进行验证
 */
#include "test_helper.hpp"
#include "control_type.hpp"
#include "ALGORITHM/svpwm.hpp"

void test_svpwm()
{
    using robot_foc::algorithm::Svpwm;
    using robot_foc::common::ErrorCode;

    // 扇区判断测试
    {
        robot_foc::common::AlphaBetaVoltage AlphaBeta{};
        AlphaBeta.alpha_volt_ = 5.0F;
        AlphaBeta.beta_volt_ = 0.0F;
        std::uint8_t sector {0}; 
        auto ret = Svpwm::svpwm_detect_sector(AlphaBeta,sector);
        std::cout<<static_cast<int>(sector);
    }
}
