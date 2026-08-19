/*
 * Unit tests for the MCP23017 switch driver (fc_mcp_switches.cpp), against the
 * scriptable Wire mock in stubs/Wire.h.  The driver's contract under test is
 * the fail-safe: every bus failure surfaces as a false return so the caller
 * reads the switches as OPEN, never last-known-state.
 */
#include <unity.h>
#include "Wire.h"
#include "Debug.h"

#include "../../../../HMTL_Fire_Control_Wickerman/fc_mcp_switches.cpp"

#define REG_IODIRA 0x00
#define REG_GPPUA  0x0C
#define REG_GPIOA  0x12

void setUp() {
    debug_log_begin_test(Unity.CurrentTestName);
    wire_mock_reset();
}
void tearDown() {}

void test_init_configures_inputs_and_pullups() {
    TEST_ASSERT_TRUE(fc_mcp_switches_init());
    TEST_ASSERT_EQUAL_HEX8(0x26, wire_mock.last_addr);
    TEST_ASSERT_EQUAL_HEX8(0xFF, wire_mock.regs[REG_IODIRA]);
    TEST_ASSERT_EQUAL_HEX8(0x0F, wire_mock.regs[REG_GPPUA]);
}

void test_init_fails_when_the_chip_does_not_ack() {
    wire_mock.end_rc = 2;              // address NACK
    uint32_t before = fc_mcp_switch_errors();
    TEST_ASSERT_FALSE(fc_mcp_switches_init());
    TEST_ASSERT_TRUE(fc_mcp_switch_errors() > before);
}

void test_read_returns_the_gpioa_byte() {
    wire_mock.regs[REG_GPIOA] = 0xF5;  // bits 1,3 low = switches 1,3 closed
    uint8_t bits = 0;
    TEST_ASSERT_TRUE(fc_mcp_switches_read(&bits));
    TEST_ASSERT_EQUAL_HEX8(0xF5, bits);
}

void test_read_fails_on_write_nack_and_counts_it() {
    wire_mock.end_rc = 2;
    uint8_t bits = 0xAA;
    uint32_t before = fc_mcp_switch_errors();
    TEST_ASSERT_FALSE(fc_mcp_switches_read(&bits));
    TEST_ASSERT_EQUAL_HEX8(0xAA, bits);              // untouched on failure
    TEST_ASSERT_EQUAL_UINT32(before + 1, fc_mcp_switch_errors());
}

void test_read_fails_on_short_read() {
    wire_mock.fail_request = true;
    uint8_t bits = 0x55;
    uint32_t before = fc_mcp_switch_errors();
    TEST_ASSERT_FALSE(fc_mcp_switches_read(&bits));
    TEST_ASSERT_EQUAL_HEX8(0x55, bits);
    TEST_ASSERT_EQUAL_UINT32(before + 1, fc_mcp_switch_errors());
}

void test_recovery_after_failures() {
    wire_mock.end_rc = 2;
    uint8_t bits = 0;
    TEST_ASSERT_FALSE(fc_mcp_switches_read(&bits));
    wire_mock.end_rc = 0;                            // bus recovers
    wire_mock.regs[REG_GPIOA] = 0x0E;                // switch 0 closed
    TEST_ASSERT_TRUE(fc_mcp_switches_read(&bits));
    TEST_ASSERT_EQUAL_HEX8(0x0E, bits);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_init_configures_inputs_and_pullups);
    RUN_TEST(test_init_fails_when_the_chip_does_not_ack);
    RUN_TEST(test_read_returns_the_gpioa_byte);
    RUN_TEST(test_read_fails_on_write_nack_and_counts_it);
    RUN_TEST(test_read_fails_on_short_read);
    RUN_TEST(test_recovery_after_failures);
    return UNITY_END();
}
