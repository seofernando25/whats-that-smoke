# SAFETY

`S0:=read-only; S1:=config/reboot; S2:=sensor-read; S3:=servo/LED; S4:=motor`

## gates

```text
default=S0
S0->S1 : explicit intent + config backup + rollback
S1->S2 : bus/device identity✓ + read-only command✓
S2->S3 : board protocol✓ + bounds✓ + STOP✓
S3->S4 : wheels-off-ground✓ + clear-zone✓ + human-present✓ + timeout✓
```

## invariants

```text
motion command => finite duration + explicit stop/finally
exception|disconnect|SIGINT => all motor PWM=0
service-stop => freeze controller > independent PCA9685 brake > terminate
aruco-lost|frame-stale>450ms|owner-disconnect => brake
unknown address/register => no write
servo bounds unknown => no pulse
camera capture => no actuation
credentials => runtime only; logs/docs/git∅
```

## abort

```text
smoke|odor|heat|brownout|unexpected-motion => power-cut > diagnosis
throttled!=0x0 => stop load; inspect supply/thermal
I2C contention/error burst => stop scans/writes; power-cycle only with approval
```
