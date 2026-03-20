dsoc-extended
=============

Fork of [dsoc](https://github.com/onnokort/dsoc) by Onno Kortmann, ported to
Python 3 and extended with remote control features.

`dsoc` - DSO *control* - is a command line utility to remotely control a
Tekway/Hantek DSO of the DSO5x02(B) series through USB.

Changes from upstream
---------------------

- Ported from Python 2 to Python 3
- Fixed USB endpoints for DSO5202B (OUT=0x02, IN=0x81)
- Added RGB565 screenshot support (800x480, as used by DSO5202B)
- Fixed bulk transfer receive for large data (screenshots, samples)
- Added remote control via button press simulation (command 0x13)

Prerequisites
-------------

Python 3 with the following libraries:

- `pyusb` - USB communication
- `numpy` - sample data scaling
- `Pillow` (PIL) - screenshot image saving

Install with: `pip install pyusb numpy Pillow`

A udev rule is provided in `99-hantek.rules` for the DSO5x02B (vendor 049f,
product 505a). Copy it to `/etc/udev/rules.d/` and reload:

    sudo cp 99-hantek.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules

USB protocol reference
----------------------

The functionality is based on the reverse-engineered USB protocol documented at:

- [mikrocontroller.net](http://www.mikrocontroller.net/articles/Hantek_Protokoll) (German)
- [elinux.org](http://elinux.org/Das_Oszi_Protocol) (English)

Usage
-----

Each sub-command has its own help page: `./dsoc <command> --help`

### Data acquisition and diagnostics

    ./dsoc ping                          # Test communication
    ./dsoc reset                         # Reset scope to defaults
    ./dsoc settings                      # Show current scope settings
    ./dsoc samples 1 -o data.txt         # Get samples from CH1
    ./dsoc samples 1 -r                  # Raw ADC values only
    ./dsoc screenshot output.png         # Save screenshot

### File system and shell access

    ./dsoc cat /etc/hostname             # Read file from scope
    ./dsoc sh ls /                       # Run command on scope (DANGEROUS!)

### Vertical controls

    ./dsoc set-vdiv 1 1.0                # Set CH1 to 1V/div
    ./dsoc set-vdiv 2 0.1               # Set CH2 to 100mV/div
    ./dsoc position 1 10                 # Move CH1 up 10 steps
    ./dsoc position 2 -5                 # Move CH2 down 5 steps
    ./dsoc reset-position 1              # Reset CH1 position to zero
    ./dsoc coupling 1 dc                 # Set CH1 coupling to DC
    ./dsoc coupling 2 ac                 # Set CH2 coupling to AC
    ./dsoc probe 1 1                     # Set CH1 probe to 1x
    ./dsoc probe 2 10                    # Set CH2 probe to 10x
    ./dsoc channel 1 on                  # Enable CH1
    ./dsoc channel 2 off                 # Disable CH2

### Timebase

    ./dsoc set-tdiv 0.001                # Set to 1ms/div
    ./dsoc set-tdiv 0.00005              # Set to 50us/div

### Trigger controls

    ./dsoc trig-level -V 0.5             # Set trigger to 0.5V
    ./dsoc trig-level 10                 # Adjust trigger up 10 steps
    ./dsoc trig-50                       # Set trigger to 50% of signal
    ./dsoc trig-mode auto                # Set trigger mode (auto/normal)
    ./dsoc trig-source ch1               # Set source (ch1/ch2/ext/ext5/ac50)
    ./dsoc trig-slope rising             # Set slope (rising/falling)
    ./dsoc trig-coupling dc              # Set coupling (dc/ac/noise/hf/lf)

### Miscellaneous

    ./dsoc beep -d 200                   # Beep for 200ms

### Global options

    ./dsoc -c <command>                  # Show USB communication on stderr
    ./dsoc -v <command>                  # Verbose output

Testing
-------

An end-to-end test suite is provided in `test_dsoc.py`. It requires a
Hantek DSO5202B connected via USB with the udev rule installed (see
Prerequisites above).

Install test dependencies:

    pip install pytest

Run the full suite:

    pytest test_dsoc.py -v --tb=short

The tests cover USB communication, settings readback, waveform samples,
screenshots, filesystem access, vertical/horizontal/trigger controls,
panel lock, and scope reset. The scope is reset to factory defaults at
the start of each session via the DEFAULT SETUP button, so prior scope
state does not affect results.

Limitations
-----------

- Remote control commands work by simulating button presses, so they take
  some time to execute (200ms per button press).
- Trigger menu commands (source, slope, mode, coupling) only work reliably
  when the trigger type is set to **Edge**. Other trigger types change the
  menu layout.
- Trigger voltage setting has limited resolution due to integer step
  quantization.

V/div valid values: 2mV, 5mV, 10mV, 20mV, 50mV, 100mV, 200mV, 500mV,
1V, 2V, 5V, 10V.

---

Original author: Onno Kortmann <onno@gmx.net>
