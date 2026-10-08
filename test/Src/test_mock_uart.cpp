/**
 * @brief MockUart 单元测试函数
 * @details 验证MockUart完整生命周期、configure校验、收发行为、缓冲区规则、deinit状态复位；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "DRIVERS/mock_uart.hpp"
#include "test_helper.hpp"
#include "COMMON/error_code.hpp"

bool test_mock_uart()
{
    using robot_foc::drivers::MockUart;
    using robot_foc::hal::Uart;
    using robot_foc::hal::UartConfig;
    using robot_foc::hal::UartDataBits;
    using robot_foc::hal::UartStopBits;
    using robot_foc::hal::UartParity;
    using robot_foc::common::ErrorCode;

    const std::size_t fail_start = test_failure_count;

    // 单元测试缓冲区容量
    constexpr std::size_t TEST_RX_CAP = 8U;
    constexpr std::size_t TEST_TX_CAP = 8U;
    MockUart<TEST_RX_CAP, TEST_TX_CAP> uart;
    std::uint8_t rx_byte{0U};

    // ========== 未初始化状态测试 ==========
    // 未init，inject_rx注入失败，不能写入接收缓存
    auto ret_inject_pre = uart.inject_rx(0xAAU);
    EXPECT(ret_inject_pre == ErrorCode::ErrorNotReady, "UART pre-init inject_rx fails");
    EXPECT(uart.rx_available() == 0U, "UART pre-init rx buffer empty");

    // 未init，send_byte发送失败
    auto ret_send1_pre = uart.send_byte(0xBBU);
    EXPECT(ret_send1_pre == ErrorCode::ErrorNotReady, "UART pre-init send_byte fails");

    // 未init，send_bytes发送失败
    const std::uint8_t dummy_data[] = {0x11,0x22};
    auto ret_sendN_pre = uart.send_bytes(dummy_data, sizeof(dummy_data));
    EXPECT(ret_sendN_pre == ErrorCode::ErrorNotReady, "UART pre-init send_bytes fails");

    // ========== configure 配置合法性校验 ==========
    // 非法波特率0，configure返回ErrorInvalidParam，原有配置保持不变
    UartConfig origin_cfg = uart.get_config();
    UartConfig bad_baud_cfg;
    bad_baud_cfg.baudrate = 0U;
    auto ret_cfg_bad = uart.configure(bad_baud_cfg);
    EXPECT(ret_cfg_bad == ErrorCode::ErrorInvalidParam, "UART configure baudrate=0 reject");
    EXPECT(uart.get_config().baudrate == origin_cfg.baudrate, "UART bad baud config unchanged");

    // 合法配置
    UartConfig valid_cfg;
    valid_cfg.baudrate = 115200U;
    auto ret_cfg_ok = uart.configure(valid_cfg);
    EXPECT(ret_cfg_ok == ErrorCode::Ok, "UART valid configure success");

    // ========== 初始化、重复初始化校验 ==========
    auto ret_init1 = uart.init();
    EXPECT(ret_init1 == ErrorCode::Ok, "UART first init succeeds");
    // 已初始化，再次init返回ErrorAlreadyInit
    auto ret_init2 = uart.init();
    EXPECT(ret_init2 == ErrorCode::ErrorAlreadyInit, "UART repeated init fails");

    // 已初始化状态下调用configure，拒绝修改，原配置保留
    UartConfig another_cfg;
    another_cfg.baudrate = 9600U;
    UartConfig after_init_cfg = uart.get_config();
    auto ret_cfg_after_init = uart.configure(another_cfg);
    EXPECT(ret_cfg_after_init == ErrorCode::ErrorAlreadyInit, "UART configure reject when initialized");
    EXPECT(uart.get_config().baudrate == after_init_cfg.baudrate, "UART config unchanged after init");

    // ========== RX接收注入测试（已初始化才可注入） ==========
    auto ret_inject_ok = uart.inject_rx(0x55U);
    EXPECT(ret_inject_ok == ErrorCode::Ok, "UART inject_rx success after init");
    EXPECT(uart.rx_available() == 1U, "UART rx available count correct");
    auto ret_read_rx = uart.read_rx(rx_byte);
    EXPECT(ret_read_rx == ErrorCode::Ok, "UART read_rx success");
    EXPECT(rx_byte == 0x55U, "UART received byte match");

    // ========== TX send_byte：缓冲区满返回ErrorBufferFull，不覆盖旧数据 ==========
    // 填满TX缓冲区
    for (std::size_t i = 0; i < TEST_TX_CAP; ++i)
    {
        auto ret_tx_fill = uart.send_byte(static_cast<std::uint8_t>(i));
        EXPECT(ret_tx_fill == ErrorCode::Ok, "UART fill tx buffer");
    }
    // 缓冲区已满，继续发送失败，缓冲区数量不变
    auto ret_tx_full = uart.send_byte(0xFFU);
    EXPECT(ret_tx_full == ErrorCode::ErrorBufferFull, "UART send_byte fail when tx full");
    EXPECT(uart.tx_available() == TEST_TX_CAP, "UART tx buffer no overwrite");

    // ========== TX send_bytes：空间不足则全部失败，不写入部分数据 ==========
    // 清空tx，准备原子性测试
    (void)uart.tx_clear();
    auto ret_tx_one = uart.send_byte(0x01U);
    EXPECT(ret_tx_one == ErrorCode::Ok, "UART send one byte for space test");
    EXPECT(uart.tx_available() == 1U, "UART tx available after one byte");
    const std::uint8_t block_data[] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};
    const std::size_t block_len = sizeof(block_data);
    // 剩余空间不足，整体失败，不写入任何字节
    auto ret_block = uart.send_bytes(block_data, block_len);
    EXPECT(ret_block == ErrorCode::ErrorBufferFull, "UART send_bytes reject when space insufficient");
    EXPECT(uart.tx_available() == 1U, "UART send_bytes no partial write");

    // ========== 制造RX溢出场景，用于deinit复位校验 ==========
    (void)uart.rx_clear();
    (void)uart.tx_clear();
    // 填满RX缓冲区触发溢出标记
    for(std::size_t i = 0; i < TEST_RX_CAP; ++i)
    {
        auto ret_rx_fill = uart.inject_rx(static_cast<std::uint8_t>(i));
        EXPECT(ret_rx_fill == ErrorCode::Ok, "UART fill rx buffer");
    }
    // 再注入1字节，触发溢出
    auto ret_rx_overflow = uart.inject_rx(0x99U);
    EXPECT(ret_rx_overflow == ErrorCode::ErrorBufferFull, "UART rx inject trigger overflow");
    EXPECT(uart.get_rx_overflow_flag(), "UART rx overflow flag set");
    EXPECT(uart.get_rx_overflow_count() > 0U, "UART rx overflow count >0");

    // ========== deinit生命周期与状态复位校验 ==========
    auto ret_deinit1 = uart.deinit();
    EXPECT(ret_deinit1 == ErrorCode::Ok, "UART deinit succeeds");
    // deinit后再次deinit返回ErrorNotReady
    auto ret_deinit2 = uart.deinit();
    EXPECT(ret_deinit2 == ErrorCode::ErrorNotReady, "UART repeated deinit fails");

    // deinit后校验：TX空、RX空、溢出标记与计数清零
    EXPECT(uart.tx_available() == 0U, "UART deinit tx buffer empty");
    EXPECT(uart.rx_available() == 0U, "UART deinit rx buffer empty");
    EXPECT(!uart.get_rx_overflow_flag(), "UART deinit overflow flag cleared");
    EXPECT(uart.get_rx_overflow_count() == 0U, "UART deinit overflow count cleared");

    // ========== deinit之后：外设操作接口失效 ==========
    auto ret_inject_post = uart.inject_rx(0xCCU);
    EXPECT(ret_inject_post == ErrorCode::ErrorNotReady, "UART post-deinit inject_rx fails");
    auto ret_send_post = uart.send_byte(0xDDU);
    EXPECT(ret_send_post == ErrorCode::ErrorNotReady, "UART post-deinit send_byte fails");

    // ========== deinit后：复用原有配置直接init，不需要重新configure ==========
    auto ret_reinit_no_cfg = uart.init();
    EXPECT(ret_reinit_no_cfg == ErrorCode::Ok, "UART re-init without re-configure after deinit");
    EXPECT(uart.get_config().baudrate == 115200U, "UART config preserved after deinit");

    // 再deinit，准备多态测试
    (void)uart.deinit();

    // ========== 基类指针多态测试（HAL抽象接口验证） ==========
    Uart<TEST_RX_CAP>* base_uart{&uart};
    // 重新配置+init，测试修改配置场景
    auto ret_reconfig = uart.configure(valid_cfg);
    EXPECT(ret_reconfig == ErrorCode::Ok, "UART re-configure after deinit");
    auto ret_reinit = uart.init();
    EXPECT(ret_reinit == ErrorCode::Ok, "UART re-init after deinit");

    auto ret_poly_send = base_uart->send_byte(0xEEU);
    EXPECT(ret_poly_send == ErrorCode::Ok, "UART polymorphic send_byte ok");
    auto ret_poly_deinit = base_uart->deinit();
    EXPECT(ret_poly_deinit == ErrorCode::Ok, "UART polymorphic deinit ok");
    
    const std::size_t fail_end = test_failure_count;
    return (fail_end == fail_start);
}