"""
Integration tests: startup sequence and ready signal.

Requires --touch-port to be specified. Tests marked @pytest.mark.hardware
are skipped otherwise.

These tests verify:
  - The device sends 'ready' within MAX_READY_WAIT seconds of connecting
  - The device periodically re-sends 'ready' (via serial_ready() in the
    HMTL message handler)
"""

import sys
import os
import time
import pytest

sys.path.insert(0, os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'HMTL', 'python')))

import hmtl.HMTLprotocol as HMTLprotocol

pytestmark = pytest.mark.hardware


def test_device_ready_on_connect(hw_client):
    """
    If the hw_client fixture was constructed successfully, wait_for_ready()
    received the 'ready' signal. This is a smoke test — if it fails, the
    device is not responding correctly on startup.
    """
    assert hw_client is not None


def test_ready_resent_periodically(hw_serial_buff):
    """
    The HMTL message handler's serial_ready() function re-sends 'ready'
    every READY_RESEND_PERIOD ms when no message is received. Drain the
    buffer, wait up to 15s, and verify a fresh 'ready' arrives.

    This confirms the device is running the main loop and not crashed.
    """
    # Drain any queued data
    while hw_serial_buff.get(wait=0.05):
        pass

    deadline = time.time() + 15.0
    while time.time() < deadline:
        item = hw_serial_buff.get(wait=1.0)
        if item and item.data == HMTLprotocol.HMTL_CONFIG_READY:
            return  # pass

    pytest.fail("Did not receive a periodic 'ready' signal within 15 seconds")


def test_device_survives_idle(hw_serial_buff, hw_client):
    """
    After 3 seconds of inactivity, the device should still respond to a poll.
    Verifies the main loop does not crash or hang.
    """
    time.sleep(3)

    # Send a poll — should get a binary HMTL response
    hw_serial_buff.write(HMTLprotocol.get_poll_msg(HMTLprotocol.BROADCAST))

    deadline = time.time() + 5.0
    while time.time() < deadline:
        item = hw_serial_buff.get(wait=0.5)
        if item and item.is_hmtl:
            return  # pass

    pytest.fail("Device did not respond to poll after 3s idle")
