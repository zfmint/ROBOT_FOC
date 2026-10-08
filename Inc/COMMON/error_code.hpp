/**
 * @file error_code.hpp
 * @brief 驱动层、业务层通用错误码定义
 * @details 跨模块统一返回状态；多种失败场景禁止使用裸bool；
 *          调试辅助函数 error_code_to_str 为inline，可直接在头文件。
 */
#ifndef ROBOT_FOC_COMMON_ERROR_CODE_HPP
#define ROBOT_FOC_COMMON_ERROR_CODE_HPP

namespace robot_foc::common
{
    /**
     * @brief 全局通用错误码
     * @note Ok代表成功，其余未各类失败状态
     */
    enum class ErrorCode
    {
        Ok,                 ///操作成功
        ErrorInvalidParam,  ///参数非法：空指针、越界、配置非法
        ErrorNotReady,      ///设备未初始化、未就绪
        ErrorAlreadyInit,   ///重复初始化
        ErrorBufferFull,    ///缓冲区已满，无法写入
        ErrorBufferEmpty,   ///缓冲区为空，无法读取
        ErrorTimeout,       ///操作超时
        ErrorHardwareFault, ///硬件故障
        ErrorBusy,          ///外设资源忙
        ErrorUnsupported    ///不支持该操作或参数
    };

    /**
     * @brief 将ErrorCode转换为可读字符串，用于日志/单元测试打印
     * @param ec 输入错误码
     * @return 静态字符串指针，生命周期全局
     */
    inline const char* error_code_to_str(ErrorCode ec) noexcept
    {
        switch (ec)
        {
        case ErrorCode::Ok:
            return "Ok";
        case ErrorCode::ErrorInvalidParam:
            return "ErrorInvalidParam";
        case ErrorCode::ErrorNotReady:
            return "ErrorNotReady";
        case ErrorCode::ErrorAlreadyInit:
            return "ErrorAlreadyInit";
        case ErrorCode::ErrorBufferFull:
            return "ErrorBufferFull";
        case ErrorCode::ErrorBufferEmpty:
            return "ErrorBufferEmpty";
        case ErrorCode::ErrorTimeout:
            return "ErrorTimeout";
        case ErrorCode::ErrorHardwareFault:
            return "ErrorHardwareFault";
        case ErrorCode::ErrorBusy:
            return "ErrorBusy";
        case ErrorCode::ErrorUnsupported:
            return "ErrorUnsupported";
        default:
            return "UnknownErrorCode";
        }
    }
} // namespace robot_foc::common


#endif // ROBOT_FOC_COMMON_ERROR_CODE_HPP
