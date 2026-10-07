/**
 * @brief MockUart 单元测试函数
 * @details 验证MockUart完整生命周期、configure校验、收发行为、缓冲区规则、deinit状态复位；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "DRIVERS/mock_uart.hpp"
#include "test_helper.hpp"

bool test_mock_uart()
{
    using robot_foc::drivers::MockUart;
    using robot_foc::hal::Uart;
    using robot_foc::hal::UartConfig;
    using robot_foc::hal::UartDataBits;
    using robot_foc::hal::UartStopBits;
    using robot_foc::hal::UartParity;

    // 单元测试缓冲区容量
    constexpr std::size_t TEST_RX_CAP = 8U;
    constexpr std::size_t TEST_TX_CAP = 8U;
    MockUart<TEST_RX_CAP, TEST_TX_CAP> uart;
    std::uint8_t rx_byte{0U};

    // ========== 未初始化状态测试 ==========
    // 未init，inject_rx注入失败，不能写入接收缓存
    EXPECT(!uart.inject_rx(0xAAU), "UART pre-init inject_rx fails");
    EXPECT(uart.rx_available() == 0U, "UART pre-init rx buffer empty");
    // 未init，send_byte发送失败
    EXPECT(!uart.send_byte(0xBBU), "UART pre-init send_byte fails");
    // 未init，send_bytes发送失败
    const std::uint8_t dummy_data[] = {0x11,0x22};
    EXPECT(!uart.send_bytes(dummy_data, sizeof(dummy_data)), "UART pre-init send_bytes fails");

    // ========== configure 配置合法性校验 ==========
    // 非法波特率0，configure返回false，原有配置保持不变
    UartConfig origin_cfg = uart.get_config();
    UartConfig bad_baud_cfg;
    bad_baud_cfg.baudrate = 0U;
    EXPECT(!uart.configure(bad_baud_cfg), "UART configure baudrate=0 reject");
    EXPECT(uart.get_config().baudrate == origin_cfg.baudrate, "UART bad baud config unchanged");

    // 合法配置
    UartConfig valid_cfg;
    valid_cfg.baudrate = 115200U;
    EXPECT(uart.configure(valid_cfg), "UART valid configure success");

    // ========== 初始化、重复初始化校验 ==========
    EXPECT(uart.init(), "UART first init succeeds");
    // 已初始化，再次init返回false
    EXPECT(!uart.init(), "UART repeated init fails");

    // 已初始化状态下调用configure，拒绝修改，原配置保留
    UartConfig another_cfg;
    another_cfg.baudrate = 9600U;
    UartConfig after_init_cfg = uart.get_config();
    EXPECT(!uart.configure(another_cfg), "UART configure reject when initialized");
    EXPECT(uart.get_config().baudrate == after_init_cfg.baudrate, "UART config unchanged after init");

    // ========== RX接收注入测试（已初始化才可注入） ==========
    EXPECT(uart.inject_rx(0x55U), "UART inject_rx success after init");
    EXPECT(uart.rx_available() == 1U, "UART rx available count correct");
    EXPECT(uart.read_rx(rx_byte), "UART read_rx success");
    EXPECT(rx_byte == 0x55U, "UART received byte match");

    // ========== TX send_byte：缓冲区满返回false，不覆盖旧数据 ==========
    // 填满TX缓冲区
    for (std::size_t i = 0; i < TEST_TX_CAP; ++i)
    {
        EXPECT(uart.send_byte(static_cast<std::uint8_t>(i)), "UART fill tx buffer");
    }
    // 缓冲区已满，继续发送失败，缓冲区数量不变
    EXPECT(!uart.send_byte(0xFFU), "UART send_byte fail when tx full");
    EXPECT(uart.tx_available() == TEST_TX_CAP, "UART tx buffer no overwrite");

    // ========== TX send_bytes：空间不足则全部失败，不写入部分数据 ==========
    // 清空tx，准备原子性测试
    (void)uart.tx_clear();
    EXPECT(uart.send_byte(0x01U), "UART send one byte for space test");
    EXPECT(uart.tx_available() == 1U, "UART tx available after one byte");
    const std::uint8_t block_data[] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80};
    const std::size_t block_len = sizeof(block_data);
    // 剩余空间不足，整体失败，不写入任何字节
    EXPECT(!uart.send_bytes(block_data, block_len), "UART send_bytes reject when space insufficient");
    EXPECT(uart.tx_available() == 1U, "UART send_bytes no partial write");

    // ========== 制造RX溢出场景，用于deinit复位校验 ==========
    (void)uart.rx_clear();
    (void)uart.tx_clear();
    // 填满RX缓冲区触发溢出标记
    for(std::size_t i = 0; i < TEST_RX_CAP; ++i)
    {
        EXPECT(uart.inject_rx(static_cast<std::uint8_t>(i)), "UART fill rx buffer");
    }
    // 再注入1字节，触发溢出
    (void)uart.inject_rx(0x99U);
    EXPECT(uart.get_rx_overflow_flag(), "UART rx overflow flag set");
    EXPECT(uart.get_rx_overflow_count() > 0U, "UART rx overflow count >0");

    // ========== deinit生命周期与状态复位校验 ==========
    EXPECT(uart.deinit(), "UART deinit succeeds");
    // deinit后再次deinit返回false
    EXPECT(!uart.deinit(), "UART repeated deinit fails");

    // deinit后校验：TX空、RX空、溢出标记与计数清零
    EXPECT(uart.tx_available() == 0U, "UART deinit tx buffer empty");
    EXPECT(uart.rx_available() == 0U, "UART deinit rx buffer empty");
    EXPECT(!uart.get_rx_overflow_flag(), "UART deinit overflow flag cleared");
    EXPECT(uart.get_rx_overflow_count() == 0U, "UART deinit overflow count cleared");

    // deinit之后，外设操作接口失效
    EXPECT(!uart.inject_rx(0xCCU), "UART post-deinit inject_rx fails");
    EXPECT(!uart.send_byte(0xDDU), "UART post-deinit send_byte fails");

    // ========== 基类指针多态测试（HAL抽象接口验证） ==========
    Uart<TEST_RX_CAP>* base_uart{&uart};
    // 先重新配置+init，才能测试多态收发
    EXPECT(uart.configure(valid_cfg), "UART re-configure after deinit");
    EXPECT(uart.init(), "UART re-init after deinit");
    EXPECT(base_uart->send_byte(0xEEU), "UART polymorphic send_byte ok");
    EXPECT(base_uart->deinit(), "UART polymorphic deinit ok");

    return true;
}