"""
Unit tests for HMTLprotocol message construction.
Pure data tests — no client, no serial, no hardware.
"""

import sys
import os
import struct
import pytest

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'HMTL', 'python')))

import hmtl.HMTLprotocol as proto


STARTCODE = 0xFC


class TestMessageHeader:
    def test_startcode_is_0xfc(self):
        msg = proto.get_value_msg(1, 0, 0)
        assert msg[0] == STARTCODE

    def test_value_msg_length_field(self):
        msg = proto.get_value_msg(1, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.length == proto.MSG_VALUE_LEN

    def test_value_msg_total_bytes(self):
        msg = proto.get_value_msg(1, 0, 0)
        assert len(msg) == proto.MSG_VALUE_LEN

    def test_poll_msg_total_bytes(self):
        msg = proto.get_poll_msg(1)
        assert len(msg) == proto.MSG_POLL_LEN

    def test_program_msg_total_bytes(self):
        msg = proto.get_program_blink_msg(1, 0, 500, (255, 0, 0), 500, (0, 0, 0))
        assert len(msg) == proto.MSG_PROGRAM_LEN


class TestAddressEncoding:
    def test_value_msg_address(self):
        msg = proto.get_value_msg(0x42, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.address == 0x42

    def test_broadcast_address(self):
        msg = proto.get_value_msg(proto.BROADCAST, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.address == proto.BROADCAST

    def test_broadcast_constant_value(self):
        assert proto.BROADCAST == 65535

    def test_address_zero(self):
        msg = proto.get_value_msg(0, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.address == 0

    def test_poll_msg_address(self):
        msg = proto.get_poll_msg(99)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.address == 99


class TestMessageTypes:
    def test_value_msg_type_is_output(self):
        msg = proto.get_value_msg(1, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.mtype == proto.MSG_TYPE_OUTPUT

    def test_poll_msg_type_is_poll(self):
        msg = proto.get_poll_msg(1)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.mtype == proto.MSG_TYPE_POLL

    def test_poll_msg_has_response_flag(self):
        msg = proto.get_poll_msg(1)
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.flags & proto.MSG_FLAG_RESPONSE

    def test_value_msg_no_response_flag(self):
        msg = proto.get_value_msg(1, 0, 0)
        hdr = proto.MsgHdr.from_data(msg)
        assert not (hdr.flags & proto.MSG_FLAG_RESPONSE)


class TestValuePayload:
    def test_value_0(self):
        msg = proto.get_value_msg(1, 0, 0)
        val = struct.unpack_from("<H", msg, proto.MSG_OUTPUT_LEN)[0]
        assert val == 0

    def test_value_255(self):
        msg = proto.get_value_msg(1, 0, 255)
        val = struct.unpack_from("<H", msg, proto.MSG_OUTPUT_LEN)[0]
        assert val == 255

    def test_value_65535(self):
        msg = proto.get_value_msg(1, 0, 65535)
        val = struct.unpack_from("<H", msg, proto.MSG_OUTPUT_LEN)[0]
        assert val == 65535

    def test_output_index_0(self):
        msg = proto.get_value_msg(1, 0, 0)
        # Output index is at MSG_BASE_LEN + 1 (the second byte of output header)
        assert msg[proto.MSG_BASE_LEN + 1] == 0

    def test_output_index_3(self):
        msg = proto.get_value_msg(1, 3, 0)
        assert msg[proto.MSG_BASE_LEN + 1] == 3


class TestBlinkProgram:
    def test_blink_msg_length(self):
        msg = proto.get_program_blink_msg(1, 0, 500, (255, 0, 0), 500, (0, 0, 0))
        assert len(msg) == proto.MSG_PROGRAM_LEN

    def test_blink_msg_address(self):
        msg = proto.get_program_blink_msg(77, 0, 500, (255, 0, 0), 500, (0, 0, 0))
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.address == 77

    def test_blink_msg_output_type_is_program(self):
        msg = proto.get_program_blink_msg(1, 0, 500, (128, 64, 32), 500, (0, 0, 0))
        hdr = proto.MsgHdr.from_data(msg)
        assert hdr.mtype == proto.MSG_TYPE_OUTPUT


class TestNoneProgram:
    def test_none_msg_length(self):
        msg = proto.get_program_none_msg(1, 0)
        assert len(msg) == proto.MSG_PROGRAM_LEN


class TestMsgConstants:
    def test_hmtl_config_ready_is_bytes(self):
        assert isinstance(proto.HMTL_CONFIG_READY, bytes)
        assert proto.HMTL_CONFIG_READY == b"ready"

    def test_hmtl_config_ack_is_ok(self):
        assert proto.HMTL_CONFIG_ACK == b"ok"

    def test_hmtl_config_fail_is_fail(self):
        assert proto.HMTL_CONFIG_FAIL == b"fail"

    def test_hmtl_terminator_length(self):
        assert len(proto.HMTL_TERMINATOR) == 4
