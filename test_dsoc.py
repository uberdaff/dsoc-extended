"""End-to-end tests for dsoc (Hantek DSO5xxx oscilloscope control).

Requires a Hantek DSO5202B connected via USB.
Run with: pytest test_dsoc.py -v --tb=short -x
"""

import os
import stat
import time

import numpy as np
import pytest
import usb.core
from PIL import Image

from dsoconn import HTDSO


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def dso():
    """Single HTDSO instance shared across all tests.

    Resets scope settings to defaults so each session starts from a
    known state (both channels enabled, default coupling, etc.).
    """
    d = HTDSO()
    d.press_button(d.BTN_DEFAULT_SETUP)
    time.sleep(3)
    d.sync()
    d.enable_channel(2, True)
    yield d
    d.reset()


@pytest.fixture(scope="class")
def reset_scope(dso):
    """Reset scope to known state before a test class."""
    dso.press_button(dso.BTN_DEFAULT_SETUP)
    time.sleep(3)
    dso.sync()
    dso.enable_channel(2, True)


# ---------------------------------------------------------------------------
# 0. USB device permissions (udev rule check)
# ---------------------------------------------------------------------------

class TestUdevAccess:
    def test_usb_device_accessible_without_root(self):
        """Verify the udev rule grants non-root access to the scope."""
        dev = usb.core.find(idVendor=0x049f, idProduct=0x505a)
        assert dev is not None, "Scope not found on USB"
        bus = dev.bus
        addr = dev.address
        devnode = f"/dev/bus/usb/{bus:03d}/{addr:03d}"
        mode = os.stat(devnode).st_mode
        others_rw = stat.S_IROTH | stat.S_IWOTH
        assert mode & others_rw == others_rw, (
            f"{devnode} is not world-readable/writable (mode {oct(mode)}). "
            "Check udev rule for 049f:505a."
        )


# ---------------------------------------------------------------------------
# 1. Basic USB communication
# ---------------------------------------------------------------------------

class TestConnection:
    def test_echo_simple(self, dso):
        assert dso.echo("PING") == b"PING"

    def test_echo_binary(self, dso):
        payload = b"\x00\xff\x80"
        assert dso.echo(payload) == payload

    def test_echo_long(self, dso):
        payload = "A" * 200
        assert dso.echo(payload) == payload.encode()

    def test_sync(self, dso):
        dso.sync()


# ---------------------------------------------------------------------------
# 2. Read scope state (no changes)
# ---------------------------------------------------------------------------

EXPECTED_KEYS = [
    "VERT-CH1-VB", "VERT-CH1-COUP", "HORIZ-TB",
    "TRIG-MODE", "TRIG-EDGE-SLOPE", "TRIG-COUP", "TRIG-VPOS",
]


class TestSettings:
    def test_settings_returns_dict(self, dso):
        result = dso.settings()
        assert isinstance(result, tuple) and len(result) == 2
        assert isinstance(result[0], dict)
        assert isinstance(result[1], (bytes, bytearray))

    def test_settings_has_expected_keys(self, dso):
        s, _ = dso.settings()
        for key in EXPECTED_KEYS:
            assert key in s, f"Missing key: {key}"

    def test_settings_values_are_tuples(self, dso):
        s, _ = dso.settings()
        for key in EXPECTED_KEYS:
            val = s[key]
            assert isinstance(val, tuple), f"{key} value is not a tuple"

    def test_settings_vdiv_in_table(self, dso):
        s, _ = dso.settings()
        vdiv = s["VERT-CH1-VB"][0]
        assert vdiv in HTDSO.VDIV_TABLE, f"VB {vdiv} not in VDIV_TABLE"

    def test_settings_coupling_valid(self, dso):
        s, _ = dso.settings()
        assert s["VERT-CH1-COUP"][0] in ("DC", "AC", "GND")

    def test_settings_trig_mode_valid(self, dso):
        s, _ = dso.settings()
        assert s["TRIG-MODE"][0] in ("auto", "normal")


# ---------------------------------------------------------------------------
# 3. Acquire waveform data
# ---------------------------------------------------------------------------

class TestSamples:
    @pytest.mark.parametrize("ch", [0, 1])
    def test_samples_channel(self, dso, ch):
        data = dso.samples(ch)
        assert isinstance(data, np.ndarray)
        assert data.dtype == np.int8
        assert len(data) > 0

    def test_samples_invalid_channel(self, dso):
        with pytest.raises(IOError):
            dso.samples(2)


# ---------------------------------------------------------------------------
# 4. Screen capture
# ---------------------------------------------------------------------------

class TestScreenshot:
    @pytest.fixture(scope="class")
    def screenshot_img(self, dso):
        """Capture one screenshot and share across assertion tests."""
        return dso.screenshot()

    def test_screenshot_returns_image(self, screenshot_img):
        assert isinstance(screenshot_img, Image.Image)

    def test_screenshot_dimensions(self, screenshot_img):
        assert screenshot_img.size == (800, 480)

    def test_screenshot_mode(self, screenshot_img):
        assert screenshot_img.mode == "RGB"

    def test_screenshot_saves_file(self, dso, tmp_path):
        outfn = str(tmp_path / "screen.png")
        dso.screenshot(outfn=outfn)
        assert os.path.isfile(outfn)
        assert os.path.getsize(outfn) > 0


# ---------------------------------------------------------------------------
# 5. Scope filesystem and commands
# ---------------------------------------------------------------------------

class TestFileAndCommand:
    def test_get_file(self, dso):
        data = dso.get_file("/logotype.dis")
        assert isinstance(data, (bytes, bytearray))
        assert len(data) > 0

    def test_command_echo(self, dso):
        result = dso.command("echo hello")
        assert b"hello" in result

    def test_command_ls(self, dso):
        result = dso.command("ls /")
        assert b"etc" in result

    def test_beep(self, dso):
        dso.beep(100)


# ---------------------------------------------------------------------------
# 6. Channel settings (uses reset_scope)
# ---------------------------------------------------------------------------

class TestVdiv:
    @pytest.fixture(autouse=True)
    def _reset(self, reset_scope):
        pass

    @pytest.mark.parametrize("vdiv", [0.01, 0.1, 1.0, 5.0])
    def test_set_vdiv_ch1(self, dso, vdiv):
        dso.set_vdiv(1, vdiv)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH1-VB"][0] == pytest.approx(vdiv)

    @pytest.mark.parametrize("vdiv", [0.05, 2.0])
    def test_set_vdiv_ch2(self, dso, vdiv):
        dso.set_vdiv(2, vdiv)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH2-VB"][0] == pytest.approx(vdiv)


class TestCouplingAndPosition:
    @pytest.fixture(autouse=True)
    def _reset(self, reset_scope):
        pass

    @pytest.mark.parametrize("mode", ["DC", "AC", "GND"])
    def test_set_coupling_ch1(self, dso, mode):
        dso.set_coupling(1, mode)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH1-COUP"][0] == mode

    @pytest.mark.parametrize("probe", [1, 10])
    def test_set_probe_ch1(self, dso, probe):
        dso.set_probe(1, probe)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH1-PROBE"][0] == probe

    def test_set_position(self, dso):
        dso.set_position(1, 5)

    def test_reset_position(self, dso):
        dso.reset_position(1)

    def test_enable_disable_channel(self, dso):
        dso.enable_channel(2, False)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH2-DISP"][0] == 0

        dso.enable_channel(2, True)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["VERT-CH2-DISP"][0] == 1


# ---------------------------------------------------------------------------
# 7. Horizontal settings (uses reset_scope)
# ---------------------------------------------------------------------------

class TestTimebase:
    @pytest.fixture(autouse=True)
    def _reset(self, reset_scope):
        pass

    @pytest.mark.parametrize("tdiv", [1e-4, 1e-3, 1e-2])
    def test_set_tdiv(self, dso, tdiv):
        dso.set_tdiv(tdiv)
        time.sleep(0.5)
        s, _ = dso.settings()
        # Find what the scope should snap to
        nearest_idx = dso._find_nearest_idx(HTDSO.TDIV_TABLE, tdiv)
        expected = HTDSO.TDIV_TABLE[nearest_idx]
        assert s["HORIZ-TB"][0] == pytest.approx(expected, rel=0.01)


# ---------------------------------------------------------------------------
# 8. Trigger settings (uses reset_scope, ensures Edge type)
# ---------------------------------------------------------------------------

class TestTriggerControls:
    @pytest.fixture(autouse=True)
    def _reset(self, reset_scope):
        pass

    @pytest.mark.parametrize("mode", ["auto", "normal"])
    def test_set_trig_mode(self, dso, mode):
        dso.set_trig_mode(mode)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["TRIG-MODE"][0] == mode

    @pytest.mark.parametrize("source", ["CH1", "CH2"])
    def test_set_trig_source(self, dso, source):
        dso.set_trig_source(source)
        time.sleep(0.5)
        s, _ = dso.settings()
        expected_idx = HTDSO.TRIG_SOURCE_TABLE.index(source)
        assert s["TRIG-SRC"][0] == expected_idx

    @pytest.mark.parametrize("slope", ["rising", "falling"])
    def test_set_trig_slope(self, dso, slope):
        dso.set_trig_slope(slope)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["TRIG-EDGE-SLOPE"][0] == slope

    @pytest.mark.parametrize("coupling", ["DC", "AC", "NoiseRej", "HFRej", "LFRej"])
    def test_set_trig_coupling(self, dso, coupling):
        dso.set_trig_coupling(coupling)
        time.sleep(0.5)
        s, _ = dso.settings()
        assert s["TRIG-COUP"][0] == coupling

    def test_trig_50pct(self, dso):
        dso.trig_50pct()

    def test_set_trig_level_relative(self, dso):
        dso.set_trig_level(3)
        dso.set_trig_level(-3)

    def test_set_trig_voltage(self, dso):
        dso.set_trig_voltage(0.5)
        time.sleep(0.5)
        s, _ = dso.settings()
        vdiv = s["VERT-CH1-VB"][0]
        probe = s["VERT-CH1-PROBE"][0]
        actual_v = s["TRIG-VPOS"][0] * vdiv * probe / HTDSO.TRIG_VPOS_SCALE
        assert actual_v == pytest.approx(0.5, abs=0.2)


# ---------------------------------------------------------------------------
# 9. Panel/acquisition control
# ---------------------------------------------------------------------------

class TestLockAndStop:
    def test_lock_unlock_panel(self, dso):
        dso.lock_panel(True)
        time.sleep(0.3)
        dso.lock_panel(False)

    def test_stop_start_acq(self, dso):
        dso.stop_acq(True)
        time.sleep(0.3)
        dso.stop_acq(False)


# ---------------------------------------------------------------------------
# 10. Reset (runs last)
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset(self, dso):
        dso.reset()
        time.sleep(3)
        dso.sync()
        assert dso.echo("OK") == b"OK"
