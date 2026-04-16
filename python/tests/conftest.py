"""
pytest fixtures for HMTL Fire Control tests.

Provides two sets of fixtures:
  - Unit test fixtures (fake_serial, fire_client): no hardware required.
    FakeSerialBuff mocks the serial layer so HMTLSerial can be exercised
    without a real serial port.
  - Integration fixtures (hw_serial_buff, hw_client): require a connected
    device specified via --touch-port. Tests using these are marked
    @pytest.mark.hardware and are skipped unless --touch-port is given.
"""

import sys
import os
import time
import pytest

# Make the HMTL python library importable from this project's python/ directory
_HMTL_PYTHON = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'HMTL', 'python'))
if _HMTL_PYTHON not in sys.path:
    sys.path.insert(0, _HMTL_PYTHON)

_FIRE_PYTHON = os.path.normpath(os.path.join(os.path.dirname(__file__), '..'))
if _FIRE_PYTHON not in sys.path:
    sys.path.insert(0, _FIRE_PYTHON)

import hmtl.HMTLprotocol as HMTLprotocol
from hmtl.InputBuffer import InputItem


# ---------------------------------------------------------------------------
# CLI options
# ---------------------------------------------------------------------------

def pytest_addoption(parser):
    parser.addoption(
        "--touch-port",
        action="store",
        default=None,
        metavar="PORT",
        help="Serial port for the touch controller (e.g. /dev/cu.usbserial-XXXX). "
             "If not provided, hardware tests are skipped.",
    )
    parser.addoption(
        "--touch-baud",
        action="store",
        default=115200,
        type=int,
        help="Baud rate for the touch controller (default: 115200).",
    )


# ---------------------------------------------------------------------------
# Hardware test marker
# ---------------------------------------------------------------------------

def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "hardware: marks tests that require a connected touch controller (--touch-port)",
    )


def pytest_collection_modifyitems(config, items):
    if config.getoption("--touch-port", default=None) is None:
        skip_hw = pytest.mark.skip(reason="--touch-port not provided")
        for item in items:
            if "hardware" in item.keywords:
                item.add_marker(skip_hw)


# ---------------------------------------------------------------------------
# Fake serial layer for unit tests
# ---------------------------------------------------------------------------

class FakeCircularBuffer:
    """
    Controllable stand-in for CircularBuffer used in unit tests.
    Tests inject InputItems directly rather than going through pyserial.
    """

    def __init__(self):
        self._items = []

    def put(self, item):
        self._items.append(item)

    def get(self, wait=None):
        if self._items:
            return self._items.pop(0)
        if wait:
            # Simulate a short timeout rather than blocking
            time.sleep(min(wait, 0.01))
        return None

    def inject_text(self, data_bytes):
        """Add a text (non-HMTL) InputItem."""
        self._items.append(InputItem(data_bytes, time.time(), is_hmtl=False))

    def inject_hmtl(self, data_bytes):
        """Add a binary HMTL InputItem."""
        self._items.append(InputItem(data_bytes, time.time(), is_hmtl=True))


class FakeSerialBuff:
    """
    Minimal mock for SerialBuffer / InputBuffer used by HMTLSerial.

    HMTLSerial calls:
      - buff.start()         -- called by __init__ before wait_for_ready()
      - buff.get(wait=N)     -- called by get_message() to read items
      - buff.write(data)     -- called by send_and_confirm() to send data
      - buff.start_time      -- attribute read by TimedLogger

    wait_for_ready() has a 0.5-second "flush" window after start() during
    which it discards all received items. We auto-inject a 'ready' signal
    once, 0.6 seconds after start() is called, so it arrives after the
    window. This means fixture setup takes ~0.6 seconds (acceptable for unit
    tests) and the timeout test (MAX_READY_WAIT=0.1s) still works because
    it times out before 0.6s.
    """

    def __init__(self):
        self.start_time = time.time()
        self._buffer = FakeCircularBuffer()
        self.written = []  # captures all write() calls for assertion
        self._started_at = None
        self._ready_injected = False

    def start(self):
        self._started_at = time.time()
        self._ready_injected = False

    def get(self, wait=None):
        # Auto-inject 'ready' once, after 0.6s from start (past the flush window)
        if (not self._ready_injected
                and self._started_at is not None
                and (time.time() - self._started_at) >= 0.6):
            self._buffer.inject_text(HMTLprotocol.HMTL_CONFIG_READY)
            self._ready_injected = True

        return self._buffer.get(wait=wait)

    def write(self, data):
        self.written.append(data)

    def inject_response(self, data_bytes):
        """Helper: add a text response item (e.g. b'ok', b'fail')."""
        self._buffer.inject_text(data_bytes)

    def inject_hmtl_response(self, data_bytes):
        """Helper: add a binary HMTL response item."""
        self._buffer.inject_hmtl(data_bytes)

    @property
    def all_written(self):
        """Return all written bytes as a single bytes object."""
        return b"".join(self.written)


# ---------------------------------------------------------------------------
# Unit test fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def fake_serial():
    """A FakeSerialBuff pre-loaded with 'ready' signals. No hardware needed."""
    return FakeSerialBuff()


@pytest.fixture
def fire_client(fake_serial):
    """
    A FireControlClient constructed with FakeSerialBuff.
    HMTLSerial.__init__ will call wait_for_ready() using the pre-loaded signals.
    """
    from fire_control.client import FireControlClient
    return FireControlClient(fake_serial)


# ---------------------------------------------------------------------------
# Hardware (integration) fixtures — session-scoped
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def touch_port(request):
    port = request.config.getoption("--touch-port")
    if port is None:
        pytest.skip("--touch-port not provided")
    return port


@pytest.fixture(scope="session")
def touch_baud(request):
    return request.config.getoption("--touch-baud")


@pytest.fixture(scope="session")
def hw_serial_buff(touch_port, touch_baud):
    """
    Session-scoped real SerialBuffer connected to the touch controller.

    Session scope is intentional: the ESP32 connected via CP2102N does NOT
    reset when the serial port is opened (unlike ATmega + DTR reset). A new
    port open would waste 10s re-waiting for 'ready'. One connection per
    test session is sufficient.
    """
    from hmtl.SerialBuffer import SerialBuffer
    buff = SerialBuffer(touch_port, baud=touch_baud, timeout=0.1, verbose=False)
    yield buff
    buff.stop()


@pytest.fixture(scope="session")
def hw_client(hw_serial_buff):
    """
    Session-scoped FireControlClient backed by a real serial connection.
    Calls all_off() on teardown as a safety measure.
    """
    from fire_control.client import FireControlClient
    client = FireControlClient(hw_serial_buff)
    yield client
    try:
        client.all_off()
    except Exception:
        pass
