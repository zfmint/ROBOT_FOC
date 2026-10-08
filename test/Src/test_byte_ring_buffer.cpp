/**
 * @brief ByteRingBuffer 环形缓冲区单元测试函数
 * @details 验证带count计数版本环形缓冲区：初始化状态、push/pop读写、peek、写满、索引环绕、clear、最小容量边界；
 *          使用EXPECT，单条失败继续跑完剩余用例，最后统一汇总失败计数
 */
#include "HAL/byte_ring_buffer.hpp"
#include "test_helper.hpp"

bool test_byte_ring_buffer()
{
    using robot_foc::hal::ByteRingBuffer;
    std::uint8_t out_val{};
    const std::size_t fail_start = test_failure_count;

    // ========== 测试1：缓冲区初始状态 ==========
    ByteRingBuffer<8> rb8;
    EXPECT(rb8.empty(), "RingBuffer initial state is empty");
    EXPECT(!rb8.full(), "RingBuffer initial state is not full");
    EXPECT(rb8.size() == 0U, "RingBuffer initial size equals zero");
    EXPECT(rb8.capacity() == 8U, "RingBuffer capacity matches template parameter");

    // ========== 测试2：基础push、pop顺序读写 ==========
    EXPECT(rb8.push(0x11), "push byte 0x11 success");
    EXPECT(rb8.push(0x22), "push byte 0x22 success");
    EXPECT(rb8.size() == 2U, "size equals 2 after two push");
    EXPECT(!rb8.empty(), "buffer not empty after push data");

    EXPECT(rb8.pop(out_val), "pop first stored byte");
    EXPECT(out_val == 0x11, "pop value matches 0x11");
    EXPECT(rb8.pop(out_val), "pop second stored byte");
    EXPECT(out_val == 0x22, "pop value matches 0x22");

    EXPECT(rb8.empty(), "buffer empty after pop all data");
    EXPECT(!rb8.pop(out_val), "pop from empty buffer returns false");

    // ========== 测试3：peek 读取队首，不消耗缓冲区数据 ==========
    ByteRingBuffer<8> rb_peek;
    EXPECT(!rb_peek.peek(out_val), "peek on empty buffer returns false");

    EXPECT(rb_peek.push(0xAA), "push byte 0xAA for peek test");
    EXPECT(rb_peek.peek(out_val), "peek get front byte success");
    EXPECT(out_val == 0xAA, "peek value matches 0xAA");
    EXPECT(rb_peek.size() == 1U, "peek will not reduce buffer size");

    EXPECT(rb_peek.pop(out_val), "pop after peek operation");
    EXPECT(out_val == 0xAA, "pop get same byte as peek");

    // ========== 测试3.1：多次peek，队首保持不变 ==========
    ByteRingBuffer<4> rb_multi_peek;
    EXPECT(rb_multi_peek.push(0x10), "multi peek push 0x10");
    EXPECT(rb_multi_peek.push(0x20), "multi peek push 0x20");
    EXPECT(rb_multi_peek.peek(out_val), "multi peek #1");
    EXPECT(out_val == 0x10, "multi peek #1 val");
    EXPECT(rb_multi_peek.peek(out_val), "multi peek #2");
    EXPECT(out_val == 0x10, "multi peek #2 val");
    EXPECT(rb_multi_peek.size() == 2U, "peek does not consume data");

    // ========== 测试4：写满缓冲区，full状态校验，满时push拒绝写入 ==========
    ByteRingBuffer<8> rb_fill;
    for(std::size_t i = 0; i < 8U; ++i)
    {
        EXPECT(rb_fill.push(static_cast<std::uint8_t>(i)), "push to fill buffer");
    }
    EXPECT(rb_fill.full(), "buffer becomes full after writing capacity bytes");
    EXPECT(rb_fill.size() == 8U, "size equals capacity when full");
    EXPECT(!rb_fill.push(0xFF), "push to full buffer returns false, no overwrite");

    // 全部读出并校验顺序
    for(std::size_t i = 0; i < 8U; ++i)
    {
        EXPECT(rb_fill.pop(out_val), "pop byte from full buffer");
        EXPECT(out_val == static_cast<std::uint8_t>(i), "pop value sequence match");
    }
    EXPECT(rb_fill.empty(), "buffer empty after pop all stored bytes");

    // ========== 测试5：索引环绕边界测试，读写触发取模环绕 ==========
    ByteRingBuffer<4> rb_wrap;
    // 写满4字节
    EXPECT(rb_wrap.push(10U), "wrap test push 10");
    EXPECT(rb_wrap.push(20U), "wrap test push 20");
    EXPECT(rb_wrap.push(30U), "wrap test push 30");
    EXPECT(rb_wrap.push(40U), "wrap test push 40");
    EXPECT(rb_wrap.full(), "buffer full before wrap read");

    // 读出2个，释放空间
    EXPECT(rb_wrap.pop(out_val), "wrap pop 10");
    EXPECT(out_val == 10U, "wrap pop value 10");
    EXPECT(rb_wrap.pop(out_val), "wrap pop 20");
    EXPECT(out_val == 20U, "wrap pop value 20");
    EXPECT(rb_wrap.size() == 2U, "remaining data count = 2");

    // 继续写入2字节，write_idx发生环绕
    EXPECT(rb_wrap.push(50U), "wrap push 50");
    EXPECT(rb_wrap.push(60U), "wrap push 60");
    EXPECT(rb_wrap.full(), "buffer full after wrap write");

    // 依次读取剩余全部数据，拆分为独立断言
    EXPECT(rb_wrap.pop(out_val), "wrap pop value 30");
    EXPECT(out_val == 30U, "wrap pop value 30");
    EXPECT(rb_wrap.pop(out_val), "wrap pop value 40");
    EXPECT(out_val == 40U, "wrap pop value 40");
    EXPECT(rb_wrap.pop(out_val), "wrap pop value 50");
    EXPECT(out_val == 50U, "wrap pop value 50");
    EXPECT(rb_wrap.pop(out_val), "wrap pop value 60");
    EXPECT(out_val == 60U, "wrap pop value 60");
    EXPECT(rb_wrap.empty(), "buffer empty after wrap read all");

    // ========== 测试6：clear清空缓冲区测试 ==========
    ByteRingBuffer<8> rb_clear;
    EXPECT(rb_clear.push(0x01), "clear test push 0x01");
    EXPECT(rb_clear.push(0x02), "clear test push 0x02");
    EXPECT(!rb_clear.empty(), "buffer has data before clear");

    rb_clear.clear();
    EXPECT(rb_clear.empty(), "buffer empty after clear");
    EXPECT(rb_clear.size() == 0U, "size reset to zero after clear");
    EXPECT(!rb_clear.pop(out_val), "pop failed after buffer cleared");

    // ========== 测试7：最小容量边界 ByteRingBuffer<2> ==========
    ByteRingBuffer<2> rb_min;
    EXPECT(rb_min.push(0x01), "min capacity push 0x01");
    EXPECT(rb_min.push(0x02), "min capacity push 0x02");
    EXPECT(rb_min.full(), "min capacity buffer full");
    EXPECT(!rb_min.push(0x03), "min capacity full push reject");

    EXPECT(rb_min.pop(out_val), "min cap pop value 0x01");
    EXPECT(out_val == 0x01, "min cap pop value 0x01");
    EXPECT(rb_min.pop(out_val), "min cap pop value 0x02");
    EXPECT(out_val == 0x02, "min cap pop value 0x02");
    EXPECT(rb_min.empty(), "min cap buffer empty after pop all");

    const std::size_t fail_end = test_failure_count;
    // 本用例内部新增失败 ==0 返回true
    return (fail_end == fail_start);
}