# This file is part of dsoc.

# dsoc is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# dsoc is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with dsoc.  If not, see <http://www.gnu.org/licenses/>.

from sys import stderr
import usb
import struct
ba=bytearray
from time import sleep
from numpy import array, int8, int16
from functools import reduce

def str2hex(s):
    return "".join("%02x " % i for i in ba(s))

# Y scale seems to be a bit off between samples and screen
# this value gives the approximate number of y divisions in the sample data.
yscale_div=10.3

# full X scale, when menu is disabled
xscale_div=19.2

def io_check(cond, s):
    if not cond:
        raise IOError(s)

class HTDSO(object):
    def __init__(self, verbose=False):
        self.verbose=verbose
        self.max_retries=5
        self.rbuf=bytearray()
        c=usb.core.find(idVendor=0x049f, idProduct=0x505a)
        self.c=c
        if not c:
            raise IOError("DSO not found.")
        if c.is_kernel_driver_active(0):
            c.detach_kernel_driver(0)
        self.sync()

    def transmit(self, vals, head):
        vals=ba(vals)
        l=len(vals)+1
        io_check(l<0x10000, "TX packet too long.")
        s=ba([head])+struct.pack("<H", l)
        s+=ba(vals)
        s.append(sum(s)&0xff)
        if self.verbose:
            print("-> DSO", str2hex(s), file=stderr)
        self.c.write(0x2, s)
        sleep(0.05)

    def recv_decode(self, r, vals, head):
        io_check(r[0]==head, "Response header does not match (0x%02x, 0x%02x)." %
                 (r[0], head))
        p_end=r[1]+256*r[2]+2
        io_check(p_end<len(r), "Partial packet.")
        io_check(r[3]==vals[0]|0x80, "Response command type does not match.")
        io_check(r[p_end]==sum(r[:p_end])&0xff, "Response checksum does not match.")
        return r[4:p_end], p_end+1

    def receive(self, vals, head):
        vals=ba(vals)
        # Keep reading until we have a complete packet
        for attempt in range(200):
            if len(self.rbuf)<4 or (len(self.rbuf)>=3 and
                    self.rbuf[1]+256*self.rbuf[2]+3 > len(self.rbuf)):
                try:
                    rvals=ba(self.c.read(0x81, 0x10000, timeout=5000))
                    self.rbuf+=rvals
                    if self.verbose:
                        print("<- DSO [%d bytes, buf now %d]" % (len(rvals), len(self.rbuf)), file=stderr)
                except usb.core.USBError:
                    if len(self.rbuf)>=4:
                        break
                    raise
            else:
                break
        if self.verbose and len(self.rbuf) <= 64:
            print(" - BUF:", str2hex(self.rbuf), file=stderr)
        elif self.verbose:
            print(" - BUF: [%d bytes]" % len(self.rbuf), file=stderr)
        try:
            r, i=self.recv_decode(self.rbuf, vals, head)
            self.rbuf=self.rbuf[i:]
            return r
        except IOError as e:
            if self.verbose:
                print("Receive decode error:", e, file=stderr)
            self.rbuf=bytearray(0)
            try:
                self.c.read(0x81, 0x10000, timeout=1000)
            except (IOError, usb.core.USBError):
                pass
            raise

    def echo(self, s):
        s=b"\x00"+s.encode() if isinstance(s, str) else b"\x00"+s
        self.transmit(s, 0x53)
        r=self.receive(s, 0x53)
        io_check(r==s[1:], "ECHO reply does not match.")
        return r

    def sync(self):
        self.rbuf=bytearray()
        for i in range(100):
            try:
                self.echo("sync")
                sleep(0.1)
                self.echo("sync")
            except (IOError, usb.core.USBError):
                continue
            break

    def retry(f):
        def w(self, *args, **kwargs):
            for i in range(self.max_retries):
                try:
                    return f(self, *args, **kwargs)
                except IOError as e:
                    if self.verbose:
                        print(
                            "Communication failed (%s), "
                            "retry no. %d" % (e, i), file=stderr)

            raise IOError("Maximum number of retries exceeded.")
        return w

    @retry
    def lock_panel(self, lock=True):
        s=ba([0x12, 0x01, bool(lock)])
        self.transmit(s, 0x53)
        r=self.receive(s, 0x53)

    @retry
    def stop_acq(self, stop=True):
        s=ba([0x12, 0x00, bool(stop)])
        self.transmit(s, 0x53)
        r=self.receive(s, 0x53)

    def dsosafe(stop=True, lock=True):
        def wrapper(f):
            def wlock(self, *args, **kwargs):
                self.stop_acq(stop)
                self.lock_panel(lock)
                try:
                    return f(self, *args, **kwargs)
                except:
                    self.sync()
                    raise
                finally:
                    self.lock_panel(False)
                    self.stop_acq(False)
            return wlock
        return wrapper


    def bulk_input(self, s, head, samplemode=False, samplechan=0x00):
        f=ba()
        chk=0
        while True:
            r=self.receive(s, head)
            if r[0]==0x01:
                if samplemode:
                    io_check(r[1]==samplechan, "Sample channel "
                             "does not match received data.")
                f+=r[1+samplemode:]
                chk+=sum(r[1:])
            elif r[0]==0x02:
                if not samplemode:
                    io_check(chk&0xff==r[1], "Bulk checksum does not match.")
                else:
                    io_check(r[1]==samplechan, "Sample channel "
                             "does not match received data.")
                return f
            else:
                io_check(0, "Invalid bulk transfer id %d." % r[0])

    @retry
    @dsosafe()
    def get_file(self, fn):
        fn=fn.encode() if isinstance(fn, str) else fn
        s=ba([0x10, 0x00])+fn
        self.transmit(s, 0x53)
        return self.bulk_input(s, 0x53)

    @retry
    @dsosafe()
    def command(self, cmd):
        cmd=cmd.encode() if isinstance(cmd, str) else cmd
        s=ba([0x11])+cmd
        self.transmit(s, 0x43)
        return self.receive(s, 0x43)

    @retry
    @dsosafe()
    def screenshot(self, outfn=None):
        from PIL import Image
        s=ba([0x20])
        self.transmit(s, 0x53)
        r=self.bulk_input(s, 0x53)
        if len(r)==800*480*2:
            # 16-bit RGB565 color data (DSO5202B and similar)
            img=Image.new("RGB", (800, 480))
            for y in range(480):
                for x in range(800):
                    idx=(x+800*y)*2
                    pixel=r[idx] | (r[idx+1]<<8)
                    red=((pixel>>11)&0x1f)<<3
                    green=((pixel>>5)&0x3f)<<2
                    blue=(pixel&0x1f)<<3
                    img.putpixel((x, y), (red, green, blue))
        elif len(r)==800*480:
            # 8-bit palette mode (original DSO5x02)
            img=Image.new("P", (800, 480))
            import palette
            pal=reduce(lambda x,y:list(x)+list(y), palette.pal)
            img.putpalette(pal)
            for y in range(480):
                for x in range(800):
                    img.putpixel((x, y), r[x+800*y])
        else:
            io_check(False, "Image size does not match (got %d bytes)." % len(r))
        if outfn:
            img.save(outfn)
        return img

    @retry
    @dsosafe()
    def beep(self, ms):
        io_check( ms<25500, "Beep length too long.")
        s=ba([0x44, ms//100])
        self.transmit(s, 0x43)
        io_check(not len(self.receive(s, 0x43)), "Beep command returned data.")

    @retry
    @dsosafe(stop=False)
    def samples(self, ch):
        io_check(ch in [0,1], "Invalid channel for sample data.")
        s=ba([0x02, 0x01, ch])
        self.transmit(s, 0x53)
        r=self.receive(s, 0x53)
        slen=struct.unpack("<L", bytes(r[1:])+b"\x00")[0]
        sdata=self.bulk_input(s, 0x53, True, ch)
        io_check(len(sdata)==slen,
            ("Length of sample data received (%d) "+
            "does not match advertised length %d.") % (len(sdata), slen))
        return array(sdata, int8)

    @retry
    @dsosafe()
    def settings(self):
        import io
        fmt=io.StringIO(self.get_file("/protocol.inf").decode())
        scopetype=self.get_file("/logotype.dis")[:-1].decode()

        from operator import iadd

        self.transmit(b"\x01", 0x53)
        s=self.receive(b"\x01", 0x53)

        coupling=["DC", "AC", "GND"]

        divy=reduce(iadd, ([.002*10**i,.005*10**i, .010*10**i] for i in
                           range(4)))

        tstates=["stop", "ready",
                 "auto", "trig'd",
                 "scan", "astop",
                 "armed"]

        tsource=["CH1", "CH2",
                 "EXT", "EXT/5",
                 "AC50"]

        ttypes=["Edge", "Video", "Pulse",
                "Slope", "O.T.", "Alt"]

        tmode=["auto", "normal"]

        tcoupling=["DC", "AC", "NoiseRej",
                   "HFRej", "LFRej"]

        tedges=["rising", "falling"]

        verti=["non-inverted", "inverted"]

        # horizontal settings do not seem to match (anymore?)
        # the horiz_scale setting is for a Hantek DSO5102B with
        # FW 120808.0, if comparing to the SysDATA v1.0 document.
        #tscales=reduce(iadd, ([4e-9*10**i,8e-9*10**i, 20e-9*10**i] for i in range(10)))+[40]
        tscales=reduce(iadd, ([2e-9*10**i, 4e-9*10**i,8e-9*10**i] for i in range(11)))

        unpack={1 : "<B", 2 : "<h", 8 : "<Q"}

        translations=[
            ("^VERT-CH.-COUP$",     lambda x : (coupling[x], "")),
            ("^VERT-CH.-VB$",       lambda x : (divy[x], "V")),
            ("^VERT-CH.-PROBE$",    lambda x : (10**x, "x")),
            ("^VERT-CH.-RPHASE$",   lambda x : (verti[x], "")),
            ("^TRIG-STATE$",        lambda x : (tstates[x], "")),
            ("^TRIG-TYPE$",         lambda x : (ttypes[x],  "")),
            ("^TRIG-MODE$",         lambda x : (tmode[x],   "")),
            ("^TRIG-COUP$",         lambda x : (tcoupling[x], "")),
            ("^TRIG-FREQUENCY$",    lambda x : (x*1e-3, "Hz")),
            ("^TRIG-HOLDTIME-MIN$", lambda x : (x*1e-12, "s")),
            ("^TRIG-HOLDTIME-MAX$", lambda x : (x*1e-12, "s")),
            ("^TRIG-HOLDTIME$",     lambda x : (x*1e-12, "s")),
            ("^TRIG-EDGE-SLOPE$",   lambda x : (tedges[x], "")),
            ("^HORIZ-TB$",          lambda x : (tscales[x], "s"))
            ]

        res={"UNIT" : (scopetype, "")}

        for i, l in enumerate(fmt):
            ls=l.split()
            name=ls[0].replace("[", "").replace("]", "")
            if name=="TOTAL":
                assert(i==0)
                continue
            elif name=="START":
                assert(i==1)
                continue
            elif name=="END":
                assert(len(s)==0)
                continue

            n=int(ls[1])

            v=struct.unpack(unpack[n], bytes(s[:n]))[0]

            for r, c in translations:
                import re
                if re.match(r, name):
                    value, comment=c(v)
                    res[name]=value, comment
                    break
            else:
                res[name]=v, ""
            s=s[n:]

        return res, s

    @retry
    def reset(self):
        """ Reset scope to initial state. """
        self.transmit(b"\x7f", 0x43)

    # Button keycodes for simulated knob turns / presses
    BTN_CH1_MENU = 0x18
    BTN_CH1_POS_DEC = 0x19
    BTN_CH1_POS_INC = 0x1A
    BTN_CH1_POS_PRESS = 0x1B
    BTN_CH1_VDIV_DEC = 0x1C
    BTN_CH1_VDIV_INC = 0x1D
    BTN_CH2_MENU = 0x1E
    BTN_CH2_POS_DEC = 0x1F
    BTN_CH2_POS_INC = 0x20
    BTN_CH2_POS_PRESS = 0x21
    BTN_CH2_VDIV_DEC = 0x22
    BTN_CH2_VDIV_INC = 0x23
    BTN_TIMEBASE_DEC = 0x28
    BTN_TIMEBASE_INC = 0x29
    BTN_TRIG_LEVEL_DEC = 0x2B
    BTN_TRIG_LEVEL_INC = 0x2C
    BTN_TRIG_LEVEL_PRESS = 0x2D
    BTN_TRIG_50PCT = 0x2E
    BTN_TRIG_MENU = 0x2A
    BTN_RUN_STOP = 0x13
    BTN_F0 = 0x00
    BTN_F1 = 0x01
    BTN_F2 = 0x02
    BTN_F3 = 0x03
    BTN_F4 = 0x04
    BTN_F5 = 0x05

    VDIV_TABLE = [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]

    TDIV_TABLE = []
    for _i in range(11):
        TDIV_TABLE.extend([2e-9*10**_i, 4e-9*10**_i, 8e-9*10**_i])

    @retry
    def press_button(self, keycode, count=1):
        for _ in range(count):
            s=ba([0x13, keycode, 0x01])
            self.transmit(s, 0x53)
            self.receive(s, 0x53)
            sleep(0.2)

    def _find_nearest_idx(self, table, value):
        """Find the index of the nearest value in a table."""
        best_idx = 0
        best_ratio = abs(table[0] / value - 1) if value != 0 else float('inf')
        for i, v in enumerate(table):
            ratio = abs(v / value - 1) if value != 0 else float('inf')
            if ratio < best_ratio:
                best_ratio = ratio
                best_idx = i
        return best_idx

    def set_vdiv(self, channel, target_v):
        """Set volt/div for channel (1 or 2) to target_v (e.g. 1.0 for 1V/div)."""
        io_check(channel in [1, 2], "Channel must be 1 or 2.")
        target_idx = self._find_nearest_idx(self.VDIV_TABLE, target_v)
        sets, _ = self.settings()
        key = "VERT-CH%d-VB" % channel
        current_v = sets[key][0]
        current_idx = self._find_nearest_idx(self.VDIV_TABLE, current_v)
        delta = target_idx - current_idx
        if channel == 1:
            btn_inc, btn_dec = self.BTN_CH1_VDIV_INC, self.BTN_CH1_VDIV_DEC
        else:
            btn_inc, btn_dec = self.BTN_CH2_VDIV_INC, self.BTN_CH2_VDIV_DEC
        if delta > 0:
            self.press_button(btn_inc, delta)
        elif delta < 0:
            self.press_button(btn_dec, -delta)

    def set_tdiv(self, target_s):
        """Set time/div to target_s seconds (e.g. 0.001 for 1ms/div)."""
        target_idx = self._find_nearest_idx(self.TDIV_TABLE, target_s)
        sets, _ = self.settings()
        current_s = sets["HORIZ-TB"][0]
        current_idx = self._find_nearest_idx(self.TDIV_TABLE, current_s)
        delta = target_idx - current_idx
        if delta > 0:
            self.press_button(self.BTN_TIMEBASE_INC, delta)
        elif delta < 0:
            self.press_button(self.BTN_TIMEBASE_DEC, -delta)

    # Trigger voltage scale: voltage = VPOS * V_div * probe / 25.0
    TRIG_VPOS_SCALE = 25.0

    TRIG_SOURCE_TABLE = ["CH1", "CH2", "EXT", "EXT/5", "AC50"]
    TRIG_SLOPE_TABLE = ["rising", "falling"]
    TRIG_COUPLING_TABLE = ["DC", "AC", "NoiseRej", "HFRej", "LFRej"]
    TRIG_MODE_TABLE = ["auto", "normal"]

    # Trigger menu F-button mapping
    TRIG_F_TYPE = BTN_F1
    TRIG_F_SOURCE = BTN_F2
    TRIG_F_SLOPE = BTN_F3
    TRIG_F_MODE = BTN_F4
    TRIG_F_COUPLING = BTN_F5

    def set_trig_level(self, steps):
        """Adjust trigger level by N steps. Positive = up, negative = down."""
        if steps > 0:
            self.press_button(self.BTN_TRIG_LEVEL_INC, steps)
        elif steps < 0:
            self.press_button(self.BTN_TRIG_LEVEL_DEC, -steps)

    def set_trig_voltage(self, voltage):
        """Set trigger level to an absolute voltage."""
        sets, _ = self.settings()
        src_idx = sets["TRIG-SRC"][0]
        # Use source channel's V/div and probe for voltage calculation
        if src_idx <= 1:
            ch = src_idx + 1
            vdiv = sets["VERT-CH%d-VB" % ch][0]
            probe = sets["VERT-CH%d-PROBE" % ch][0]
        else:
            # EXT/EXT5/AC50 - use CH1 settings as reference
            vdiv = sets["VERT-CH1-VB"][0]
            probe = sets["VERT-CH1-PROBE"][0]
        current_vpos = sets["TRIG-VPOS"][0]
        target_vpos = int(round(voltage * self.TRIG_VPOS_SCALE / (vdiv * probe)))
        delta = target_vpos - current_vpos
        self.set_trig_level(delta)

    def trig_50pct(self):
        """Set trigger level to 50% of the signal."""
        self.press_button(self.BTN_TRIG_50PCT)

    def _trig_menu_cycle(self, f_button, table, settings_key, target):
        """Cycle a trigger menu setting to the target value."""
        sets, _ = self.settings()
        current = sets[settings_key][0]
        if isinstance(current, str):
            current_idx = table.index(current)
        else:
            current_idx = current
        target_idx = table.index(target)
        if current_idx == target_idx:
            return
        presses = (target_idx - current_idx) % len(table)
        self.press_button(self.BTN_TRIG_MENU)
        sleep(0.3)
        self.press_button(f_button, presses)
        sleep(0.3)

    def set_trig_mode(self, mode):
        """Set trigger mode to 'auto' or 'normal'."""
        mode = mode.lower()
        io_check(mode in self.TRIG_MODE_TABLE,
                 "Mode must be 'auto' or 'normal'.")
        self._trig_menu_cycle(self.TRIG_F_MODE, self.TRIG_MODE_TABLE,
                              "TRIG-MODE", mode)

    def set_trig_source(self, source):
        """Set trigger source to CH1, CH2, EXT, EXT/5, or AC50."""
        source = source.upper()
        io_check(source in self.TRIG_SOURCE_TABLE,
                 "Source must be one of: %s" % self.TRIG_SOURCE_TABLE)
        # TRIG-SRC is a numeric index
        sets, _ = self.settings()
        current_idx = sets["TRIG-SRC"][0]
        target_idx = self.TRIG_SOURCE_TABLE.index(source)
        if current_idx == target_idx:
            return
        presses = (target_idx - current_idx) % len(self.TRIG_SOURCE_TABLE)
        self.press_button(self.BTN_TRIG_MENU)
        sleep(0.3)
        self.press_button(self.TRIG_F_SOURCE, presses)
        sleep(0.3)

    def set_trig_slope(self, slope):
        """Set trigger edge slope to 'rising' or 'falling'."""
        slope = slope.lower()
        io_check(slope in self.TRIG_SLOPE_TABLE,
                 "Slope must be 'rising' or 'falling'.")
        self._trig_menu_cycle(self.TRIG_F_SLOPE, self.TRIG_SLOPE_TABLE,
                              "TRIG-EDGE-SLOPE", slope)

    def set_trig_coupling(self, coupling):
        """Set trigger coupling to DC, AC, NoiseRej, HFRej, or LFRej."""
        # Normalize user-friendly names
        coupling_map = {
            "DC": "DC", "AC": "AC",
            "NOISEREJ": "NoiseRej", "NOISE": "NoiseRej", "NR": "NoiseRej",
            "HFREJ": "HFRej", "HF": "HFRej",
            "LFREJ": "LFRej", "LF": "LFRej",
        }
        key = coupling.upper().replace(" ", "").replace("_", "")
        io_check(key in coupling_map,
                 "Coupling must be one of: DC, AC, NoiseRej, HFRej, LFRej")
        target = coupling_map[key]
        self._trig_menu_cycle(self.TRIG_F_COUPLING, self.TRIG_COUPLING_TABLE,
                              "TRIG-COUP", target)

    def set_position(self, channel, steps):
        """Adjust vertical position for channel (1 or 2) by N steps.
        Positive = up, negative = down."""
        io_check(channel in [1, 2], "Channel must be 1 or 2.")
        if channel == 1:
            btn_inc, btn_dec = self.BTN_CH1_POS_INC, self.BTN_CH1_POS_DEC
        else:
            btn_inc, btn_dec = self.BTN_CH2_POS_INC, self.BTN_CH2_POS_DEC
        if steps > 0:
            self.press_button(btn_inc, steps)
        elif steps < 0:
            self.press_button(btn_dec, -steps)

    def reset_position(self, channel):
        """Reset vertical position for channel to zero (press the knob)."""
        io_check(channel in [1, 2], "Channel must be 1 or 2.")
        if channel == 1:
            self.press_button(self.BTN_CH1_POS_PRESS)
        else:
            self.press_button(self.BTN_CH2_POS_PRESS)

    COUPLING_TABLE = ["DC", "AC", "GND"]

    def set_coupling(self, channel, mode):
        """Set coupling for channel (1 or 2) to DC, AC, or GND."""
        io_check(channel in [1, 2], "Channel must be 1 or 2.")
        mode = mode.upper()
        io_check(mode in self.COUPLING_TABLE,
                 "Coupling must be DC, AC, or GND.")
        sets, _ = self.settings()
        current = sets["VERT-CH%d-COUP" % channel][0]
        if current == mode:
            return
        target_idx = self.COUPLING_TABLE.index(mode)
        current_idx = self.COUPLING_TABLE.index(current)
        presses = (target_idx - current_idx) % 3
        # Open channel menu, then press F0 (Coupling) to cycle
        btn_menu = self.BTN_CH1_MENU if channel == 1 else self.BTN_CH2_MENU
        self.press_button(btn_menu)
        sleep(0.3)
        self.press_button(self.BTN_F0, presses)
        sleep(0.3)

    def enable_channel(self, channel, enable=True):
        """Enable or disable a channel."""
        io_check(channel in [1, 2], "Channel must be 1 or 2.")
        sets, _ = self.settings()
        currently_on = bool(sets["VERT-CH%d-DISP" % channel][0])
        if currently_on == enable:
            return
        btn_menu = self.BTN_CH1_MENU if channel == 1 else self.BTN_CH2_MENU
        if enable:
            # Pressing menu button when channel is off turns it on
            self.press_button(btn_menu)
            sleep(0.3)
        else:
            # Press twice: first opens menu, second disables channel
            self.press_button(btn_menu)
            sleep(0.3)
            self.press_button(btn_menu)
            sleep(0.3)

if __name__=="__main__":
    h=HTDSO()
