"""
Integration tests: poofer command dispatch over RS485.

WARNING: Run these tests with the RS485 bus DISCONNECTED from real poofers
(or with poofers physically absent). These tests send RS485 commands to
poofer addresses but do NOT verify physical ignition.

Tests verify:
  - The device handles value messages without crashing
  - Debug output at the configured DEBUG_LEVEL matches expectations
  - The device remains responsive after sending commands
"""

import sys
import os
import time
import pytest

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'HMTL', 'python')))

import hmtl.HMTLprotocol as HMTLprotocol

pytestmark = pytest.mark.hardware

# Addresses from platformio.ini build flags
POOFER1_ADDRESS = 66
POOFER2_ADDRESS = 69
LIGHTS_ADDRESS = 67

# Settle time after sending a command before checking output
SETTLE_S = 0.3


def _drain(hw_serial_buff, timeout=0.5):
    """Collect available items for `timeout` seconds."""
    items = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        item = hw_serial_buff.get(wait=0.05)
        if item:
            items.append(item)
    return items


def _still_alive(hw_serial_buff, timeout=5.0):
    """Send a poll and return True if an HMTL response arrives."""
    hw_serial_buff.write(HMTLprotocol.get_poll_msg(HMTLprotocol.BROADCAST))
    deadline = time.time() + timeout
    while time.time() < deadline:
        item = hw_serial_buff.get(wait=0.5)
        if item and item.is_hmtl:
            return True
    return False


class TestValueCommandHandling:
    def test_value_zero_does_not_crash(self, hw_client, hw_serial_buff):
        """Sending value=0 (safe/off) to poofer address must not crash the device."""
        hw_client.send_value(POOFER1_ADDRESS, 0, 0)
        time.sleep(SETTLE_S)
        assert _still_alive(hw_serial_buff), "Device unresponsive after send_value(0)"

    def test_value_to_poofer2_does_not_crash(self, hw_client, hw_serial_buff):
        hw_client.send_value(POOFER2_ADDRESS, 0, 0)
        time.sleep(SETTLE_S)
        assert _still_alive(hw_serial_buff), "Device unresponsive after send_value to poofer2"

    def test_value_broadcast_does_not_crash(self, hw_client, hw_serial_buff):
        hw_client.send_value(HMTLprotocol.BROADCAST, 0, 0)
        time.sleep(SETTLE_S)
        assert _still_alive(hw_serial_buff), "Device unresponsive after broadcast send_value"

    def test_all_off_does_not_crash(self, hw_client, hw_serial_buff):
        """all_off() sends 8 messages; device must handle all of them."""
        hw_client.all_off()
        time.sleep(0.5)
        assert _still_alive(hw_serial_buff), "Device unresponsive after all_off()"


class TestDebugOutputLevels:
    def test_debug3_connect_lines_suppressed(self, hw_client, hw_serial_buff):
        """
        At DEBUG_LEVEL_CONNECT=2, the sendValue/sendTimed DEBUG3 lines
        in Fire_Control_Connect.cpp must NOT appear in serial output.

        If these lines appear, it means the firmware was compiled with
        DEBUG_LEVEL_CONNECT >= 3, which contradicts the platformio.ini config.
        """
        _drain(hw_serial_buff, timeout=0.2)  # clear buffer
        hw_client.send_value(POOFER1_ADDRESS, 0, 0)
        time.sleep(SETTLE_S)
        lines = _drain(hw_serial_buff, timeout=0.5)
        text_lines = [i.data for i in lines if not i.is_hmtl and i.data]

        for line in text_lines:
            assert b"sendValue:" not in line, \
                f"sendValue: debug line appeared (DEBUG_LEVEL_CONNECT must be < 3): {line}"
            assert b"sendTimed:" not in line, \
                f"sendTimed: debug line appeared (DEBUG_LEVEL_CONNECT must be < 3): {line}"

    def test_startup_debug2_lines_not_present_during_run(self, hw_serial_buff):
        """
        During normal operation, startup strings (DEBUG2) should NOT appear
        unless the device just rebooted. If they appear here, the device
        rebooted unexpectedly.
        """
        _drain(hw_serial_buff, timeout=0.2)
        time.sleep(1.0)
        lines = _drain(hw_serial_buff, timeout=0.5)
        text_lines = [i.data for i in lines if not i.is_hmtl and i.data]

        for line in text_lines:
            assert b"HMTL Fire Control Initializing" not in line, \
                "Startup message appeared — device may have rebooted unexpectedly"


class TestDeviceResiliency:
    def test_rapid_commands_handled(self, hw_client, hw_serial_buff):
        """
        Sending 10 commands in quick succession should not crash the device.
        """
        for _ in range(10):
            hw_client.send_value(POOFER1_ADDRESS, 0, 0)
        time.sleep(0.5)
        assert _still_alive(hw_serial_buff), "Device unresponsive after rapid commands"

    def test_response_after_inactivity(self, hw_serial_buff):
        """
        After 5 seconds of inactivity, the device should still respond.
        """
        time.sleep(5)
        assert _still_alive(hw_serial_buff), "Device unresponsive after 5s inactivity"
