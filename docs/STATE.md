# STATE

`snapshot:=2026-08-29; mode:=observe-only`

## ✓

```text
host.model = Raspberry Pi 4 Model B Rev 1.5
host.arch = aarch64
host.os = Debian GNU/Linux 13/trixie
host.kernel = 6.18.34+rpt-rpi-v8
host.ram ~= 2GiB
host.disk.root ~= 14GiB; used~=49%
host.temp = 37..41C
host.throttle = 0x0
power.mobile = Freenove 2-cell pack -> S1 -> Pi GPIO/header; verified boot + throttle=0x0
host.failed_system_units = 0
ssh.wifi = advanced@192.168.8.170:22 ✓
ssh.ethernet = advanced@192.168.8.169:22 ✓ when cabled
wifi.profile = FillMeUpLink; autoconnect=yes; gateway=192.168.8.1
uv = /usr/local/bin/uv@0.12.7 ✓
camera.sensor = ov5647@0x36 ✓
camera.capture = 2592x1944/JPEG ✓
camera.mount_rotation = hflip+vflip = 180deg
pcb.label = V3.0
```

## ×|?

```text
i2c.header = /dev/i2c-1 ✓
i2c.car = {0x40:PCA9685,0x48:ADC,0x70:PCA9685-all-call} ✓
i2c.aux = {0,10,20,21,22}; camera/display domain
spi = ×; /dev/spidev* absent
car.controllers = detected/read-address-only ✓
motors+servos = dashboard-controlled; polarity corrected empirically
ultrasonic/line = untested
battery.pack ~= 8.85V observed; two-cell operation confirmed
led.spi-gpio10 = ×; 8×RGB sequence sent; no visible output
led.pwm-gpio18 = ×; 8×RGB sequence sent; no visible output
led.status = unsupported/unresolved on PCB V3; dashboard scope∅
Freenove software = absent at initial audit
PCB-v3 protocol/pin map = ?; public FNK0043 docs describe v1/v2 only
```

## next

```text
1 identify PCB-v3 authoritative schematic/protocol
2 establish STOP primitive + wheel-off-ground test fixture
3 test sensors -> servo -> motors, one subsystem/step
4 each effect -> verify + append snapshot
```
