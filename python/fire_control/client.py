import time
import sys
import os

# Allow running from the python/ directory without installation
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'HMTL', 'python'))

import hmtl.HMTLprotocol as HMTLprotocol
from hmtl.HMTLSerial import HMTLSerial


class FireControlClient:
    """
    Client for communicating with the HMTL Fire Control touch controller.

    Uses dependency injection for the serial buffer so it can be tested
    without hardware by passing a FakeSerialBuff instead of a real SerialBuffer.

    Usage (hardware):
        from hmtl.SerialBuffer import SerialBuffer
        buff = SerialBuffer('/dev/cu.usbserial-XXXX', baud=115200)
        client = FireControlClient(buff)

    Usage (unit tests):
        from tests.conftest import FakeSerialBuff
        buff = FakeSerialBuff()
        client = FireControlClient(buff)
    """

    # Known poofer addresses from platformio.ini
    POOFER1_ADDRESS = 66
    POOFER2_ADDRESS = 69
    LIGHTS_ADDRESS = 67

    def __init__(self, serial_buff):
        """
        :param serial_buff: An InputBuffer-compatible object (real SerialBuffer or FakeSerialBuff).
                            Must implement: start(), get(wait=N), write(data), start_time attribute.
                            FireControlClient calls HMTLSerial(serial_buff) which calls start()
                            and wait_for_ready() internally.
        """
        self.hmtl = HMTLSerial(serial_buff, verbose=False)

    def send_value(self, address, output, value):
        """Send an HMTL value message to set a single output."""
        msg = HMTLprotocol.get_value_msg(address, output, value)
        self.hmtl.serial.write(msg)

    def send_poll(self, address=HMTLprotocol.BROADCAST):
        """Send an HMTL poll message and wait for a binary response."""
        msg = HMTLprotocol.get_poll_msg(address)
        self.hmtl.serial.write(msg)
        return self._wait_for_hmtl_response(timeout=5.0)

    def igniter_on(self, address, output):
        """Turn on an igniter output (value=255)."""
        self.send_value(address, output, 255)

    def igniter_off(self, address, output):
        """Turn off an igniter output (value=0)."""
        self.send_value(address, output, 0)

    def all_off(self):
        """Safety: send value=0 to all outputs on all known poofer addresses."""
        for addr in [self.POOFER1_ADDRESS, self.POOFER2_ADDRESS]:
            for out in range(4):
                self.send_value(addr, out, 0)

    def drain(self, timeout=0.5):
        """
        Collect all available serial items for up to `timeout` seconds.
        Returns a list of InputItems.
        """
        items = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            item = self.hmtl.get_message(timeout=0.05)
            if item:
                items.append(item)
        return items

    def contains_text(self, items, pattern):
        """Return True if any text item in the list contains `pattern` (bytes)."""
        return any(
            not item.is_hmtl and item.data and pattern in item.data
            for item in items
        )

    def _wait_for_hmtl_response(self, timeout=5.0):
        """Collect items until an HMTL binary response arrives or timeout."""
        deadline = time.time() + timeout
        items = []
        while time.time() < deadline:
            item = self.hmtl.get_message(timeout=0.1)
            if item:
                items.append(item)
                if item.is_hmtl:
                    break
        return items
