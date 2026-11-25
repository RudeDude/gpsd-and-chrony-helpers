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

The default cache for `/dev/ttyACM0` is `aiding-cache.json` in this directory. Any other device uses `aiding-cache-<device>.json`.

The cache contains latitude, longitude, and height. The default names are listed in `.gitignore`. A path passed with `-f` is ignored only if you add that name yourself. Keep cache files out of git.

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

The account that runs the script needs read access to those logs, and to `chrony.conf` if you want the suggested `refclock` line. The script has no chrony Python package of its own.

## License

MIT. See [LICENSE](LICENSE).
