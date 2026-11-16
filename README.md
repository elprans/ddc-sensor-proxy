# ddc-sensor-proxy

Exposes a monitor's DDC/CI ambient light sensor over D-Bus as
`net.hadess.SensorProxy`, the interface of
[iio-sensor-proxy](https://gitlab.freedesktop.org/hadess/iio-sensor-proxy).
KWin (and anything else that speaks that API) then treats it as a regular
ambient light sensor and can offer automatic brightness on a desktop.

Only the light sensor part of the interface is implemented; accelerometer
and proximity always report "not present".  It replaces iio-sensor-proxy,
so the two can't run at the same time.

## Supported monitors

Developed against the Dell U4025QW, which reports its sensor in VCP `0x67`
while auto-brightness (VCP `0x66` == `0x82`) is enabled.  The reading keeps
working in HDR mode, where the OSD greys the option out.

The sensor only samples in HDR mode if auto-brightness was on when the
monitor entered HDR.  Turning it on over DDC (or finding it off, which the
daemon warns about) while already in HDR leaves the reading frozen; turn
auto-brightness on with HDR off, then re-enable HDR.

Other monitors
with a DDC/CI light sensor should work with the right `--vcp`,
`--enable-vcp`, `--offset` and `--scale`.

## Building and installing

```sh
cargo build --release
install -Dm755 target/release/ddc-sensor-proxy /usr/bin/ddc-sensor-proxy
install -Dm644 net.hadess.SensorProxy.conf /usr/share/dbus-1/system.d/net.hadess.SensorProxy.conf
install -Dm644 ddc-sensor-proxy.service /usr/lib/systemd/system/ddc-sensor-proxy.service
systemctl enable --now ddc-sensor-proxy
```

The daemon runs as root (it needs `/dev/i2c-*` and the system bus name);
the unit sandboxes it down to the i2c devices, the bus and netlink.

## Usage

```
ddc-sensor-proxy [--model U4025QW | --device /dev/i2c-N] [options]
```

The monitor is located by matching `--model` against the EDID name of the
connected DRM connectors, or given directly with `--device`.  See `--help`
for the rest.

To keep wakeups down, the poll interval adapts: it starts at `--interval`
(2 s) after a change and doubles while the reading is steady, up to
`--max-interval` (10 s) while a client has claimed the sensor and
`--idle-interval` (60 s) otherwise.  DRM hotplug events trigger an
immediate re-probe.

## License

MIT
