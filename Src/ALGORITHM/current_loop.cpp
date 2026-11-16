/**
 * @file current_loop.cpp
 * @brief FOC dq轴电流闭环控制器实现
 * @details
 * 内部链路：
 * 1. preview：Id/Iq预计算原始Vd/Vq
 * 2. inverse_park → VoltageLimiter矢量限幅 → park回dq
 * 3. prepare_update 根据限幅状态计算下一积分状态
 * 4. commit 固化PI积分
 * 5. 逆Park后电压送入SVPWM输出占空比
 */
#include "ALGORITHM/current_loop.hpp"

#include "units.hpp"
#include "error_code.hpp"
#include "control_type.hpp"
#include "ALGORITHM/svpwm.hpp"
#include "ALGORITHM/pid.hpp"

namespace robot_foc::algorithm
{
    [[nodiscard]] common::ErrorCode CurrentLoop::configure(
        const PiConfig& id_config,
        const PiConfig& iq_config
    ) noexcept
    {
        auto ret_id = id_cfg_.configure(id_config);
        auto ret_iq = iq_cfg_.configure(iq_config);
        if ((ret_id == common::ErrorCode::Ok) && (ret_iq == common::ErrorCode::Ok))
        {
            return common::ErrorCode::Ok;
        }
        return common::ErrorCode::ErrorNotReady;
    }
} // namespace robot::foc::algorithm
