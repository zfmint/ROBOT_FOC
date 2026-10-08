/**
 * @file test_error_code.cpp
 * @brief ErrorCode 全局错误码单元测试
 * @details 验证错误码枚举相等性、不等性、字符串转换函数 error_code_to_str 基础逻辑；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "COMMON/error_code.hpp"
#include "test_helper.hpp"
#include <cstring>
#include <cstdio>

bool test_error_code()
{
    using namespace robot_foc::common;
    const std::size_t fail_start = test_failure_count;

    // ========== 枚举相等 / 不等校验 ==========
    EXPECT(ErrorCode::Ok == ErrorCode::Ok, "ErrorCode::Ok equal check");
    EXPECT(!(ErrorCode::Ok == ErrorCode::ErrorNotReady), "ErrorCode Ok != ErrorNotReady");
    EXPECT(ErrorCode::ErrorInvalidParam != ErrorCode::ErrorBufferFull, "ErrorCode different enum not equal");

    // ========== 错误码字符串映射校验 ==========
    const char* s_ok = error_code_to_str(ErrorCode::Ok);
    printf("ErrorCode::Ok -> |%s|\n", s_ok);
    EXPECT(strcmp(s_ok, "Ok") == 0, "Ok string match");

    const char* s_inv_param = error_code_to_str(ErrorCode::ErrorInvalidParam);
    printf("ErrorCode::ErrorInvalidParam -> |%s|\n", s_inv_param);
    EXPECT(strcmp(s_inv_param, "ErrorInvalidParam") == 0, "ErrorInvalidParam string");

    const char* s_not_ready = error_code_to_str(ErrorCode::ErrorNotReady);
    printf("ErrorCode::ErrorNotReady -> |%s|\n", s_not_ready);
    EXPECT(strcmp(s_not_ready, "ErrorNotReady") == 0, "ErrorNotReady string");

    const char* s_already_init = error_code_to_str(ErrorCode::ErrorAlreadyInit);
    printf("ErrorCode::ErrorAlreadyInit -> |%s|\n", s_already_init);
    EXPECT(strcmp(s_already_init, "ErrorAlreadyInit") == 0, "ErrorAlreadyInit string");

    const char* s_buf_full = error_code_to_str(ErrorCode::ErrorBufferFull);
    printf("ErrorCode::ErrorBufferFull -> |%s|\n", s_buf_full);
    EXPECT(strcmp(s_buf_full, "ErrorBufferFull") == 0, "ErrorBufferFull string");

    const char* s_buf_empty = error_code_to_str(ErrorCode::ErrorBufferEmpty);
    printf("ErrorCode::ErrorBufferEmpty -> |%s|\n", s_buf_empty);
    EXPECT(strcmp(s_buf_empty, "ErrorBufferEmpty") == 0, "ErrorBufferEmpty string");

    const char* s_timeout = error_code_to_str(ErrorCode::ErrorTimeout);
    printf("ErrorCode::ErrorTimeout -> |%s|\n", s_timeout);
    EXPECT(strcmp(s_timeout, "ErrorTimeout") == 0, "ErrorTimeout string");

    const char* s_hw_fault = error_code_to_str(ErrorCode::ErrorHardwareFault);
    printf("ErrorCode::ErrorHardwareFault -> |%s|\n", s_hw_fault);
    EXPECT(strcmp(s_hw_fault, "ErrorHardwareFault") == 0, "ErrorHardwareFault string");

    const char* s_busy = error_code_to_str(ErrorCode::ErrorBusy);
    printf("ErrorCode::ErrorBusy -> |%s|\n", s_busy);
    EXPECT(strcmp(s_busy, "ErrorBusy") == 0, "ErrorBusy string");

    const char* s_unsupported = error_code_to_str(ErrorCode::ErrorUnsupported);
    printf("ErrorCode::ErrorUnsupported -> |%s|\n", s_unsupported);
    EXPECT(strcmp(s_unsupported, "ErrorUnsupported") == 0, "ErrorUnsupported string");

    // ========== 兜底未知枚举值测试 ==========
    auto fake_code = static_cast<ErrorCode>(999);
    const char* s_unknown = error_code_to_str(fake_code);
    printf("Unknown code(999) -> |%s|\n", s_unknown);
    EXPECT(strcmp(s_unknown, "UnknownErrorCode") == 0, "Unknown error code fallback string");

    const std::size_t fail_end = test_failure_count;
    return (fail_end == fail_start);
}