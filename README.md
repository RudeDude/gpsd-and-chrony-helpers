# gpsd and chrony helpers

Python tools for sending u-blox aiding data through [gpsd](https://gpsd.io/), and for choosing chrony `offset` and `delay` on USB GPS refclocks.

## Utilities

### `aid-ini.py`

Send one position and time hint to the receiver with `ubxtool`.

u-blox 7 (UBX protocol below 15) gets `UBX-AID-INI`. u-blox 8 and 9 get `UBX-MGA-INI-TIME_UTC` and then `UBX-MGA-INI-POS_LLH`. The time fields come from the system clock at the moment the script runs. gpsd must already have the receiver open.

```bash
./aid-ini.py LAT LON
./aid-ini.py --device ttyACM1 --alt 180 LAT LON
```

`LAT` and `LON` are degrees. Other options:

| Option | Meaning | Default |
| --- | --- | --- |
| `--device` | `ttyACM0`, `ttyACM1`, or a `/dev` path | `ttyACM0` |
| `--alt` | Height above the ellipsoid, meters | `180` |
| `--pos-acc` | Position accuracy, meters | `100` |
| `--time-acc` | Time accuracy, seconds | `10` |
| `--leap` | GPS-UTC leap seconds | `18` |
| `--protver` | ubxtool protocol version | the version gpsd reports |

### `aid-cache.py`

Save or restore u-blox aiding data for one gpsd device.

`save` polls the receiver and writes a JSON cache. On u-blox 7 that is health, UTC, and ionosphere parameters (`UBX-AID-HUI`), plus almanac, ephemeris, and AssistNow Autonomous. On u-blox 8 and 9 it is the navigation database (`UBX-MGA-DBD`). When gpsd has a 2D or 3D fix for that device, the file also stores that position.

`load` sends a fresh time hint at the saved position by running `aid-ini.py`, then uploads records that are still worth sending. Age for everything except the almanac is the age of the cache file:

| Record | Sent when |
| --- | --- |
| Ephemeris (`UBX-AID-EPH`) | the file is under 3 hours old |
| AssistNow Autonomous (`UBX-AID-AOP`) | the file is under 3 days old |
| Health (`UBX-AID-HUI`) and navigation database (`UBX-MGA-DBD`) | the file is under 180 days old |
| Almanac (`UBX-AID-ALM`) | the almanac week is within 26 weeks of the current GPS week |

Older ephemeris and AssistNow Autonomous records stay in the file and are skipped. A restored u-blox 8/9 database is uploaded whole; the receiver drops ephemeris older than a few hours.

```bash
./aid-cache.py save
./aid-cache.py load
./aid-cache.py save --device ttyACM1
./aid-cache.py load -f /path/to/cache.json
```

The default cache is `<device>-cache.json` in this directory. `/dev/ttyACM0` writes `ttyACM0-cache.json`. `/dev/ttyACM1` writes `ttyACM1-cache.json`. The name is the device's basename, so a short name such as `ttyACM1` uses the same file as `/dev/ttyACM1`.

The cache contains latitude, longitude, and height. `.gitignore` ignores `*-cache.json`. A path passed with `-f` is ignored when its name matches that pattern. Keep cache files out of git.

### `chrony-gps.py`

Read chrony's `refclocks.log` and `tracking.log` and recommend a refclock `offset` and `delay` for each `GPS*` refid. When it can read `chrony.conf`, it also prints a `refclock` line with those two values filled in. The script never writes the config.

A refclock sample counts while all of these are true:

- an NTP server is the selected source
- the tracking offset is under 1 ms, so the local clock already matches NTP
- the sample is past the warmup after a chronyd start (default 15 minutes, seen as tracking ref `0.0.0.0`)
- the sample is past the settle time after a refclock stops being the selected source (default 90 seconds)

Samples from an earlier `offset` setting in the same log are dropped when cooked-minus-raw steps by more than 2 ms. The recommended offset is the negation of the median raw sample in the latest regime, rounded to 0.1 ms. `delay / 2` is the +/- bar chrony uses when it picks a source. It is the chosen percentile of the filtered residuals after the new offset, rounded up to the next millisecond. The default percentile is 99. `poll`, `filter`, `precision`, and `maxdispersion` are left unchanged.

```bash
./chrony-gps.py
./chrony-gps.py --startup 0 --min-span 0
./chrony-gps.py --cover max --refid GPS2
```

| Option | Meaning | Default |
| --- | --- | --- |
| `--log-dir` | Directory with `refclocks.log` and `tracking.log` | `/var/log/chrony` |
| `--chrony-conf` | Read-only source for the printed `refclock` line | `/etc/chrony/chrony.conf` |
| `--startup` | Seconds to skip after each chronyd start | `900` |
| `--settle` | Seconds to skip after a refclock hands the clock back to NTP | `90` |
| `--max-offset` | Skip samples while the tracking offset is at least this many seconds | `0.001` |
| `--cover` | Percentile of the filtered residual that `delay / 2` must cover, or `max` | `99` |
| `--min-span` | Clean-window seconds required before status is `ok` | `7200` |
| `--min-samples` | Clean raw samples required before status is `ok` | `100` |
| `--min-filtered` | Clean filtered samples required before status is `ok` | `20` |
| `--refid` | Limit the report to this refid; repeat for more than one | every `GPS*` refid |
| `--include-pps` | Also report `PPS*` refids | off |
| `--include-rotated` | Also read `refclocks.log.N` and `tracking.log.N` | off |
| `--strict` | Exit 2 unless every reported refid has status `ok` | off |

A shorter window still prints a preview. Status stays `preliminary` until the sample and span minimums are met.

### `chrony-temp.py`

Recommend a chrony `tempcomp` line from the system-clock rate and a temperature sensor. The script prints the line and writes a points file in the working directory. It does not edit `chrony.conf`.

Chrony accepts two forms. The quadratic line is:

```
tempcomp <sensor> <interval> <T0> <k0> <k1> <k2>
```

`comp = k0 + (T - T0) * k1 + (T - T0)^2 * k2`, with `T` in the units of the sensor file. Linux sysfs temperatures are millidegrees Celsius. Positive `comp` speeds the clock up. Chrony ignores a reading outside ±10 ppm, so the fit removes the average clock rate and keeps only the temperature shape. The average rate stays in chrony's own frequency tracking.

The points file is the same curve sampled every 0.25 °C, with a bin median used where a temperature step has enough samples. Chrony interpolates that file. The file is numbers only.

```bash
./chrony-temp.py --sensors
./chrony-temp.py --collect 7200 --sensor /sys/class/thermal/thermal_zone0/temp
./chrony-temp.py
```

`--sensors` reads `/sys/class/thermal` and `/sys/class/hwmon` and marks board and CPU-package sensors as the ones to try first. Wifi, disk, and battery sensors are listed as distant from the oscillator.

`--collect` is the first insertion, before `tempcomp.log` exists. It samples the chosen sensor and `chronyc tracking` for the given number of seconds and appends `chrony-temp-samples.csv`. If a `tempcomp` line is already active, each sample subtracts the compensation chrony is applying so the fit is not stacked on top of it. The temperature has to move during the collection. A machine sitting at one temperature cannot show the curve.

After that line is installed, add `tempcomp` to the log directive and run `./chrony-temp.py` with no `--collect`. The refit then uses `tempcomp.log` paired with `tracking.log`. A temperature reading counts while an NTP server is selected, the tracking offset and frequency skew are inside the limits, and the reading is past the warmup after an unsynchronised sample. Each later run rewrites the points file from those logs.

| Option | Meaning | Default |
| --- | --- | --- |
| `--sensors` | List temperature files and which ones are worth trying | off |
| `--collect` | Sample the sensor and `chronyc tracking` for this many seconds, then fit | off |
| `--sensor` | Temperature file for `--collect` | required with `--collect` |
| `--source` | `auto`, `chrony`, or `preliminary` | `auto` |
| `--samples` | Preliminary CSV path | `./chrony-temp-samples.csv` |
| `--points` | Points-file path | `./chrony-tempcomp-<sensor>.points` |
| `--log-dir` | Directory with `tempcomp.log` and `tracking.log` | `/var/log/chrony` |
| `--chrony-conf` | Read-only source for the active `tempcomp` line | `/etc/chrony/chrony.conf` |
| `--interval` | Update interval in the suggested line, seconds | the configured interval, else 60 |
| `--step` | Points-file step, millidegrees | `250` |
| `--margin` | Degrees to extend the points file past the observed range | `2` |
| `--include-rotated` | Also read `tempcomp.log.N` and `tracking.log.N` | off |
| `--min-span` | Seconds required before status is `ok` | `7200` |
| `--min-samples` | Samples required before status is `ok` | `20` |
| `--min-temp-span` | Degrees of temperature span required before status is `ok` | `2` |
| `--strict` | Exit 2 unless status is `ok` | off |

Status stays `preliminary` until the sample, time, and temperature-span bars are all met. A curve that barely moves is `flat`.

`chrony-temp-samples.csv` and `chrony-tempcomp-*.points` are listed in `.gitignore`.

### `holdover-estimate.py`

Estimate how far the system clock would drift after every NTP server and refclock stops. chronyd would keep the last frequency. A `tempcomp` line would keep running, because it does not need those sources.

The script reads `chronyc tracking`, `tracking.log`, `tempcomp.log`, and the active `tempcomp` and `maxclockerror` lines. It prints three growth figures:

- the point error, which is the current offset plus the residual frequency
- the measured bound, which is the root distance plus the live skew and residual frequency
- chrony's own bound, which adds `maxclockerror` (1 ppm unless `chrony.conf` sets it)

It also reports how far the frequency estimate moved in `tracking.log` at each horizon. In holdover those corrections stop, so that movement becomes time error. A second table shows the extra rate if `tempcomp` is turned off and the sensor then moves.

```bash
./holdover-estimate.py
./holdover-estimate.py --include-rotated
```

## Python setup

Python 3.10 or newer. The `float | None` annotation in `chrony-gps.py` needs that version.

No packages are installed from pip. The scripts use the standard library. `aid-ini.py` and `aid-cache.py` also import `gpsdev.py` from this directory. That module turns a short device name into a gpsd path, reads the u-blox protocol version from gpsd, and builds the `ubxtool` command. Run the scripts with `python3` from any working directory. Python puts the script's own directory on the module path, so it finds `gpsdev.py` beside them.

On Debian and Ubuntu:

```bash
sudo apt install python3 gpsd gpsd-clients chrony
```

`gpsd` provides the daemon and `ubxtool`. `gpsd-clients` provides `gpspipe`, which the aiding scripts use to read the device list and the latest fix.

Before `aid-ini.py` or `aid-cache.py`, gpsd must already be running and must already have the receiver open. The user that runs them needs permission to talk to gpsd. `ubxtool` and `gpspipe` do that over the gpsd socket.

`chrony-gps.py` reads logs chrony writes when `chrony.conf` contains:

```
log tracking refclocks
logdir /var/log/chrony
```

`chrony-temp.py` also reads `tempcomp.log` once the log line includes `tempcomp`:

```
log tracking refclocks tempcomp
```

The account that runs the scripts needs read access to those logs, and to `chrony.conf` if you want the suggested lines. The scripts have no chrony Python package of their own. `chrony-temp.py --collect` also runs `chronyc tracking`.

## License

MIT. See [LICENSE](LICENSE).
