"""
Unit tests for FireControlClient. No hardware required.

All tests use the fake_serial / fire_client fixtures from conftest.py.
"""

import struct
import sys
import os
import pytest

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'HMTL', 'python')))
sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..')))

import hmtl.HMTLprotocol as HMTLprotocol
from hmtl.HMTLSerial import HMTLConfigException
from fire_control.client import FireControlClient


class TestWaitForReady:
    def test_client_constructs_when_ready_received(self, fire_client):
        """If wait_for_ready() succeeded, FireControlClient construction completed."""
        assert fire_client is not None

    def test_timeout_raises_when_no_ready_signal(self, fake_serial):
        """wait_for_ready() must raise after MAX_READY_WAIT if no 'ready' arrives."""
        # Clear the pre-loaded ready signals
        fake_serial._buffer._items.clear()

        import hmtl.HMTLSerial as mod
        orig = mod.HMTLSerial.MAX_READY_WAIT
        mod.HMTLSerial.MAX_READY_WAIT = 0.1  # speed up the test
        try:
            with pytest.raises(Exception, match="Timed out"):
                FireControlClient(fake_serial)
        finally:
            mod.HMTLSerial.MAX_READY_WAIT = orig


class TestSendValue:
    def test_writes_bytes_to_serial(self, fire_client, fake_serial):
        """send_value() must write a valid HMTL value message."""
        fire_client.send_value(66, 1, 255)
        expected = HMTLprotocol.get_value_msg(66, 1, 255)
        assert expected in fake_serial.all_written

    def test_message_starts_with_startcode(self, fire_client, fake_serial):
        fire_client.send_value(66, 0, 0)
        assert fake_serial.all_written[0:1] == b'\xfc'

    def test_correct_address_encoded(self, fire_client, fake_serial):
        """The HMTL address field (bytes 6–7, little-endian uint16) must match."""
        fire_client.send_value(0x42, 0, 0)
        addr = struct.unpack_from("<H", fake_serial.all_written, 6)[0]
        assert addr == 0x42

    def test_broadcast_address_encoded(self, fire_client, fake_serial):
        fire_client.send_value(HMTLprotocol.BROADCAST, 0, 0)
        addr = struct.unpack_from("<H", fake_serial.all_written, 6)[0]
        assert addr == HMTLprotocol.BROADCAST

    def test_value_encoded_in_payload(self, fire_client, fake_serial):
        """The value field follows the output header (at offset MSG_OUTPUT_LEN)."""
        fire_client.send_value(66, 0, 200)
        val = struct.unpack_from("<H", fake_serial.all_written, HMTLprotocol.MSG_OUTPUT_LEN)[0]
        assert val == 200

    def test_correct_message_length(self, fire_client, fake_serial):
        fire_client.send_value(1, 0, 0)
        assert len(fake_serial.all_written) == HMTLprotocol.MSG_VALUE_LEN

    def test_multiple_sends_accumulate(self, fire_client, fake_serial):
        fire_client.send_value(66, 0, 0)
        fire_client.send_value(69, 0, 0)
        assert len(fake_serial.written) == 2


class TestIgniterControl:
    def test_igniter_on_sends_value_255(self, fire_client, fake_serial):
        fire_client.igniter_on(66, 0)
        val = struct.unpack_from("<H", fake_serial.all_written, HMTLprotocol.MSG_OUTPUT_LEN)[0]
        assert val == 255

    def test_igniter_off_sends_value_0(self, fire_client, fake_serial):
        fire_client.igniter_off(66, 0)
        val = struct.unpack_from("<H", fake_serial.all_written, HMTLprotocol.MSG_OUTPUT_LEN)[0]
        assert val == 0

    def test_igniter_on_correct_address(self, fire_client, fake_serial):
        fire_client.igniter_on(69, 2)
        addr = struct.unpack_from("<H", fake_serial.all_written, 6)[0]
        assert addr == 69


class TestAllOff:
    def test_all_off_sends_multiple_messages(self, fire_client, fake_serial):
        """all_off() sends value=0 to 4 outputs × 2 poofer addresses = 8 messages."""
        fire_client.all_off()
        assert len(fake_serial.written) == 8

    def test_all_off_all_values_are_zero(self, fire_client, fake_serial):
        fire_client.all_off()
        written = fake_serial.all_written
        msg_len = HMTLprotocol.MSG_VALUE_LEN
        for i in range(8):
            val = struct.unpack_from("<H", written, i * msg_len + HMTLprotocol.MSG_OUTPUT_LEN)[0]
            assert val == 0, f"Message {i} has value {val}, expected 0"


class TestSendPoll:
    def test_poll_writes_poll_message(self, fire_client, fake_serial):
        # No HMTL response injected — _wait_for_hmtl_response will time out
        # but send_poll should still have written the poll bytes
        import hmtl.HMTLSerial as mod
        # Patch to short wait so test is fast
        fire_client._wait_for_hmtl_response = lambda timeout=5.0: []
        fire_client.send_poll(HMTLprotocol.BROADCAST)
        expected = HMTLprotocol.get_poll_msg(HMTLprotocol.BROADCAST)
        assert expected in fake_serial.all_written


class TestSendAndConfirm:
    def test_ack_returns_true(self, fire_client, fake_serial):
        """send_and_confirm returns True when the device sends b'ok'."""
        fake_serial.inject_response(HMTLprotocol.HMTL_CONFIG_ACK)
        result = fire_client.hmtl.send_and_confirm(b"test", terminated=False, timeout=0.5)
        assert result is True

    def test_fail_raises_hmtl_exception(self, fire_client, fake_serial):
        """send_and_confirm raises HMTLConfigException when device sends b'fail'."""
        fake_serial.inject_response(HMTLprotocol.HMTL_CONFIG_FAIL)
        with pytest.raises(HMTLConfigException):
            fire_client.hmtl.send_and_confirm(b"test", terminated=False, timeout=0.5)

    def test_timeout_raises_exception(self, fire_client):
        """send_and_confirm raises after timeout if no ACK/FAIL received."""
        with pytest.raises(Exception, match="Timed out"):
            fire_client.hmtl.send_and_confirm(b"test", terminated=False, timeout=0.05)

    def test_terminated_appends_terminator(self, fire_client, fake_serial):
        """With terminated=True, HMTL_TERMINATOR is appended to the written data."""
        fake_serial.inject_response(HMTLprotocol.HMTL_CONFIG_ACK)
        fire_client.hmtl.send_and_confirm(b"data", terminated=True, timeout=0.5)
        assert HMTLprotocol.HMTL_TERMINATOR in fake_serial.all_written


class TestContainsText:
    def test_match_found(self, fire_client, fake_serial):
        from hmtl.InputBuffer import InputItem
        items = [InputItem(b"POOFERS ENABLED", 0, is_hmtl=False)]
        assert fire_client.contains_text(items, b"POOFERS") is True

    def test_match_not_found(self, fire_client):
        from hmtl.InputBuffer import InputItem
        items = [InputItem(b"other text", 0, is_hmtl=False)]
        assert fire_client.contains_text(items, b"POOFERS") is False

    def test_hmtl_items_ignored(self, fire_client, fake_serial):
        """Binary HMTL items are not searched for text patterns."""
        msg = HMTLprotocol.get_value_msg(1, 0, 0)
        from hmtl.InputBuffer import InputItem
        items = [InputItem(msg, 0, is_hmtl=True)]
        assert fire_client.contains_text(items, msg[:4]) is False
