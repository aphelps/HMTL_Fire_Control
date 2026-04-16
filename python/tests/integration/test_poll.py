"""
Integration tests: HMTL poll message round-trip.

Verifies that the device correctly handles an HMTL POLL message and
returns a valid binary poll response with the expected object type and
a non-zero, non-broadcast address.
"""

import sys
import os
import time
import pytest

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'HMTL', 'python')))

import hmtl.HMTLprotocol as HMTLprotocol

pytestmark = pytest.mark.hardware

# OBJECT_TYPE=5 in platformio.ini (touchcontroller environment)
EXPECTED_OBJECT_TYPE = 5

# Timeout for waiting for poll response
POLL_TIMEOUT = 5.0


def _send_poll_and_wait(hw_serial_buff, address=HMTLprotocol.BROADCAST, timeout=POLL_TIMEOUT):
    """Helper: send a poll and collect items until an HMTL response arrives."""
    hw_serial_buff.write(HMTLprotocol.get_poll_msg(address))
    deadline = time.time() + timeout
    items = []
    while time.time() < deadline:
        item = hw_serial_buff.get(wait=0.5)
        if item:
            items.append(item)
            if item.is_hmtl:
                return items
    return items


def test_poll_broadcast_returns_hmtl_response(hw_serial_buff):
    """
    A broadcast POLL must elicit a binary HMTL response (starts with 0xFC).
    """
    items = _send_poll_and_wait(hw_serial_buff)
    hmtl_items = [i for i in items if i.is_hmtl]
    assert hmtl_items, "No binary HMTL poll response received within 5s"


def test_poll_response_is_poll_type(hw_serial_buff):
    """
    The binary response must have mtype == MSG_TYPE_POLL.
    """
    items = _send_poll_and_wait(hw_serial_buff)
    for item in items:
        if item.is_hmtl:
            hdr = HMTLprotocol.MsgHdr.from_data(item.data)
            assert hdr.mtype == HMTLprotocol.MSG_TYPE_POLL
            return
    pytest.fail("No parseable HMTL poll response")


def test_poll_response_has_response_flag(hw_serial_buff):
    """
    The response header must have MSG_FLAG_RESPONSE set.
    """
    items = _send_poll_and_wait(hw_serial_buff)
    for item in items:
        if item.is_hmtl:
            hdr = HMTLprotocol.MsgHdr.from_data(item.data)
            assert hdr.flags & HMTLprotocol.MSG_FLAG_RESPONSE
            return
    pytest.fail("No HMTL poll response with RESPONSE flag")


def test_poll_response_object_type(hw_serial_buff):
    """
    The PollHdr inside the response must report OBJECT_TYPE=5 (touch controller).
    """
    items = _send_poll_and_wait(hw_serial_buff)
    for item in items:
        if item.is_hmtl:
            _, last_hdr = HMTLprotocol.decode_msg(item.data)
            if hasattr(last_hdr, 'object_type'):
                assert last_hdr.object_type == EXPECTED_OBJECT_TYPE, \
                    f"Expected object_type={EXPECTED_OBJECT_TYPE}, got {last_hdr.object_type}"
                return
    pytest.skip("Poll response did not include a PollHdr with object_type (may need firmware update)")


def test_poll_response_address_is_valid(hw_serial_buff):
    """
    The device's configured address must be non-zero and non-broadcast.
    """
    items = _send_poll_and_wait(hw_serial_buff)
    for item in items:
        if item.is_hmtl:
            _, last_hdr = HMTLprotocol.decode_msg(item.data)
            if hasattr(last_hdr, 'address'):
                assert last_hdr.address != 0, "Device address is 0 (unconfigured)"
                assert last_hdr.address != HMTLprotocol.BROADCAST, \
                    "Device address is BROADCAST (unconfigured)"
                return
    pytest.skip("Poll response did not include address field")


def test_sequential_polls_both_respond(hw_serial_buff):
    """
    Two consecutive polls should both receive responses. Verifies the
    device handles repeated messages without crashing or getting stuck.
    """
    items1 = _send_poll_and_wait(hw_serial_buff)
    time.sleep(0.2)
    items2 = _send_poll_and_wait(hw_serial_buff)

    hmtl1 = any(i.is_hmtl for i in items1)
    hmtl2 = any(i.is_hmtl for i in items2)

    assert hmtl1, "First poll got no response"
    assert hmtl2, "Second poll got no response — device may have crashed"
