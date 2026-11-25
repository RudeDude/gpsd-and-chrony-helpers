#!/usr/bin/env python3
"""Recommend a chrony tempcomp line from clock rate and temperature.

Two stages:

- Before tempcomp is logged, --collect reads one sensor and chronyc
  tracking and stores <sensor>-samples.csv in the working directory.
  The sensor name is the last two parts of its path, so two collections
  can run at once. That file is enough to insert a first tempcomp line.
- A run with no flags reads every sensor that has data. That is the
  tempcomp logs, including rotated logs, plus *-samples.csv and
  *-comp.points files in the working directory. Each sensor gets its
  own section and its own config line.

The suggested line is the quadratic form:

    tempcomp <sensor> <interval> <T0> <k0> <k1> <k2>

comp = k0 + (T - T0) * k1 + (T - T0)^2 * k2

T is in the sensor file's own units. On Linux sysfs that is millidegrees
Celsius. Positive comp speeds the clock up. Chrony ignores a reading
whose comp is outside ±10 ppm, so the average clock rate is left in
chrony's frequency tracking and only the temperature shape is fitted.

The script also writes a points file in the working directory and prints
the three-argument form that uses it. Chrony interpolates that file.
The file is numbers only: chrony rejects comments and blank lines in it.

The script prints the suggestion. It does not write chrony.conf.
"""

import argparse
import csv
import ipaddress
import math
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

DATE_RE = re.compile(r"\d{4}-\d{2}-\d{2}")

# A preview needs a real temperature change, not only a long sit at one point.
MIN_FIT = 12
MIN_TEMP_PREVIEW = 0.5
# Chrony drops any compensation outside this range.
COMP_LIMIT = 10.0
# Below this peak-to-peak, the curve does not move the clock.
FLAT_PPM = 0.05

BOARD_NAMES = ("pch", "acpitz", "nct", "it87", "it86", "systin", "thinkpad")
PACKAGE_NAMES = (
    "coretemp",
    "k10temp",
    "zenpower",
    "x86_pkg",
    "pkg",
    "package",
    "cputin",
    "cpu_thermal",
    "soc",
)
DISTANT_NAMES = (
    "nvme",
    "wifi",
    "iwlwifi",
    "ath1",
    "gpu",
    "radeon",
    "amdgpu",
    "nouveau",
    "battery",
    "charger",
    "dimm",
    "sodimm",
)

KIND_NOTE = {
    "board": "Chipset or board sensor. A reasonable first sensor to try.",
    "package": "CPU package. It also moves with CPU load, which the crystal may not follow.",
    "other": "No standard name. Compare it with the clock before relying on it.",
    "distant": "Sits away from the board oscillator.",
    "unreadable": "No reading.",
}


@dataclass
class Track:
    t: datetime
    ref: str
    freq: float
    skew: float
    offset: float
    leap: str


@dataclass
class Reading:
    t: datetime
    temp: float
    comp: float
    freq: float
    skew: float
    offset: float
    ref: str
    leap: str


@dataclass
class Directive:
    sensor: str
    interval: float
    t0: float | None = None
    k0: float | None = None
    k1: float | None = None
    k2: float | None = None
    points_path: str | None = None
    points: list | None = None
    raw: str = ""


@dataclass
class Sensor:
    path: Path
    kind: str
    reading: float | None
    detail: str
    resolved: Path


def parse_stamp(date_s, time_s):
    """Chrony logs are UTC. A naive conversion would shift the pairing."""
    text = f"{date_s} {time_s}"
    if "." in time_s:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S.%f")
    else:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(tzinfo=timezone.utc)


def fmt_time(stamp):
    return stamp.strftime("%Y-%m-%d %H:%M:%SZ")


def fmt_dur(seconds):
    seconds = max(0, int(round(seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}h {minutes:02d}m"
    if minutes:
        return f"{minutes}m {secs:02d}s"
    return f"{secs}s"


def fmt_temp(milli):
    return f"{milli / 1000.0:.1f} °C"


def fmt_ppm(value):
    return f"{value:.3f} ppm"


def fmt_coeff(value):
    text = f"{value:.12f}".rstrip("0").rstrip(".")
    if text in ("", "-0", "-"):
        return "0.0"
    if "." not in text:
        text += ".0"
    return text


def fmt_interval(seconds):
    if abs(seconds - round(seconds)) < 1e-6:
        return str(int(round(seconds)))
    return f"{seconds:.1f}"


def log_files(directory, stem, rotated):
    current = directory / f"{stem}.log"
    files = []
    if rotated:
        numbered = []
        for path in directory.glob(f"{stem}.log.*"):
            suffix = path.name.split(".log.", 1)[1]
            if suffix.isdigit():
                numbered.append((int(suffix), path))
        files.extend(path for _, path in sorted(numbered, reverse=True))
    if current.exists():
        files.append(current)
    return files


def read_lines(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        sys.exit(f"cannot read {path}: {exc}")


def is_ip(text):
    try:
        ipaddress.ip_address(text)
    except ValueError:
        return False
    return True


def is_ntp(ref):
    if not is_ip(ref):
        return False
    return not ipaddress.ip_address(ref).is_unspecified


def is_unspecified(ref):
    if not is_ip(ref):
        return False
    return ipaddress.ip_address(ref).is_unspecified


def poly_at(temp, t0, k0, k1, k2):
    delta = temp - t0
    return k0 + delta * k1 + delta * delta * k2


def solve3(rows):
    """Solve a 3x3 system. Each row is [a, b, c, rhs]."""
    m = [row[:] for row in rows]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-18:
            raise ValueError("singular")
        m[col], m[pivot] = m[pivot], m[col]
        scale = m[col][col]
        for col2 in range(col, 4):
            m[col][col2] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = m[row][col]
            for col2 in range(col, 4):
                m[row][col2] -= factor * m[col][col2]
    return m[0][3], m[1][3], m[2][3]


def fit_quadratic(temps, values):
    """Fit values as k0 + k1*(T-T0) + k2*(T-T0)^2. T0 is the rounded median."""
    t0 = float(round(statistics.median(temps)))
    xs = [(temp - t0) / 1000.0 for temp in temps]
    s0 = float(len(xs))
    s1 = math.fsum(xs)
    s2 = math.fsum(x * x for x in xs)
    s3 = math.fsum(x * x * x for x in xs)
    s4 = math.fsum(x * x * x * x for x in xs)
    y0 = math.fsum(values)
    y1 = math.fsum(x * y for x, y in zip(xs, values))
    y2 = math.fsum(x * x * y for x, y in zip(xs, values))
    quad, linear, const = solve3(
        [
            [s4, s3, s2, y2],
            [s3, s2, s1, y1],
            [s2, s1, s0, y0],
        ]
    )
    # x is degrees relative to T0, so scale back to millidegrees.
    return t0, const, linear / 1000.0, quad / 1_000_000.0


def interpolate(points, temp):
    """Linear interpolation, matching chrony's tempcomp points walk."""
    if len(points) < 2:
        raise ValueError("need two points")
    upper = 1
    while upper + 1 < len(points) and points[upper][0] < temp:
        upper += 1
    t1, c1 = points[upper - 1]
    t2, c2 = points[upper]
    if t2 == t1:
        return c1
    return (temp - t1) / (t2 - t1) * (c2 - c1) + c1


def directive_comp(directive, temp):
    if directive.points:
        return interpolate(directive.points, temp)
    return poly_at(temp, directive.t0, directive.k0, directive.k1, directive.k2)


def load_points(path):
    points = []
    try:
        lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        return None, str(exc)
    for line in lines:
        body = line.strip()
        if not body or body.startswith("#"):
            continue
        parts = body.split()
        if len(parts) < 2:
            continue
        try:
            points.append((float(parts[0]), float(parts[1])))
        except ValueError:
            continue
    points.sort(key=lambda item: item[0])
    if len(points) < 2:
        return None, f"{path} has fewer than 2 points"
    return points, None


def parse_directive(line):
    body = line.split("#", 1)[0].strip()
    parts = body.split()
    if len(parts) < 4 or parts[0] != "tempcomp":
        return None
    sensor = parts[1]
    try:
        interval = float(parts[2])
    except ValueError:
        return None
    rest = parts[3:]
    raw = " ".join(parts)
    if len(rest) >= 4:
        try:
            t0, k0, k1, k2 = (float(piece) for piece in rest[:4])
        except ValueError:
            return None
        return Directive(sensor, interval, t0, k0, k1, k2, raw=raw)
    points, err = load_points(rest[0])
    directive = Directive(
        sensor, interval, points_path=rest[0], points=points, raw=raw
    )
    directive.error = err
    return directive


def load_directives(path):
    if path is None or not path.exists():
        return [], f"{path} is not readable" if path else "no chrony.conf"
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        return [], str(exc)
    found = []
    for line in lines:
        directive = parse_directive(line)
        if directive is not None:
            found.append(directive)
    return found, None


def active_directive(path):
    found, err = load_directives(path)
    if err:
        return None, err
    if not found:
        return None, None
    return found[-1], None if len(found) == 1 else f"{len(found)} tempcomp lines; using the last"


def load_tracking(paths):
    tracks = []
    for path in paths:
        for line in read_lines(path):
            parts = line.split()
            if len(parts) < 8 or not DATE_RE.fullmatch(parts[0]):
                continue
            try:
                tracks.append(
                    Track(
                        parse_stamp(parts[0], parts[1]),
                        parts[2],
                        float(parts[4]),
                        float(parts[5]),
                        float(parts[6]),
                        parts[7],
                    )
                )
            except ValueError:
                continue
    tracks.sort(key=lambda tr: tr.t)
    return tracks


def load_tempcomp(paths):
    rows = []
    for path in paths:
        for line in read_lines(path):
            parts = line.split()
            if len(parts) < 4 or not DATE_RE.fullmatch(parts[0]):
                continue
            try:
                rows.append(
                    (
                        parse_stamp(parts[0], parts[1]),
                        float(parts[2]),
                        float(parts[3]),
                    )
                )
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def _bisect_right(times, stamp):
    lo, hi = 0, len(times)
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] <= stamp:
            lo = mid + 1
        else:
            hi = mid
    return lo


def startup_windows(tracks, startup_s):
    windows = []
    if startup_s <= 0:
        return windows
    for tr in tracks:
        if is_unspecified(tr.ref) or tr.leap == "?":
            windows.append((tr.t, tr.t + timedelta(seconds=startup_s)))
    return windows


def in_windows(stamp, windows):
    for start, end in windows:
        if start <= stamp < end:
            return True
    return False


def pair_readings(temps, tracks, max_gap):
    """Attach the latest tracking sample at or before each temperature."""
    if not tracks:
        return [], list(temps)
    times = [tr.t for tr in tracks]
    paired = []
    gaps = []
    for stamp, temp, comp in temps:
        idx = _bisect_right(times, stamp) - 1
        if idx < 0:
            gaps.append((stamp, temp, comp))
            continue
        tr = tracks[idx]
        if (stamp - tr.t).total_seconds() > max_gap:
            gaps.append((stamp, temp, comp))
            continue
        paired.append(
            Reading(stamp, temp, comp, tr.freq, tr.skew, tr.offset, tr.ref, tr.leap)
        )
    return paired, gaps


def classify(reading, windows, args):
    if reading.leap == "?" or is_unspecified(reading.ref):
        return "unsynced"
    if not is_ntp(reading.ref) and not args.include_refclock:
        return "refclock"
    if in_windows(reading.t, windows):
        return "warmup"
    if abs(reading.offset) >= args.max_offset:
        return "offset"
    if abs(reading.skew) >= args.max_skew:
        return "skew"
    return "ok"


def empty_drops():
    return {
        "unsynced": 0,
        "refclock": 0,
        "warmup": 0,
        "offset": 0,
        "skew": 0,
        "gap": 0,
        "outlier": 0,
    }


def sensor_slug(path):
    parts = Path(path).parts
    if len(parts) >= 2:
        text = f"{parts[-2]}-{parts[-1]}"
    else:
        text = Path(path).name
    cleaned = []
    for ch in text:
        if ch.isalnum() or ch in "._-":
            cleaned.append(ch)
        else:
            cleaned.append("-")
    slug = "".join(cleaned).strip("-")
    return slug or "sensor"


def default_samples_path(sensor):
    return Path.cwd() / f"{sensor_slug(sensor)}-samples.csv"


def default_points_path(sensor):
    return Path.cwd() / f"{sensor_slug(sensor)}-comp.points"


def resolve_output_paths(args, directive):
    """Pick the preliminary CSV from the sensor when --samples was omitted."""
    sensor = args.sensor
    if not sensor and directive is not None:
        sensor = directive.sensor
    if args.samples is None:
        if sensor:
            args.samples = default_samples_path(sensor)
        else:
            args.samples = Path.cwd() / "sensor-samples.csv"


def read_sysfs_temp(path):
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return None, str(exc)
    piece = text.strip().split()
    if not piece:
        return None, "empty"
    try:
        return float(piece[0]), None
    except ValueError:
        return None, "not a number"


def kind_of(detail):
    text = detail.lower()
    if any(name in text for name in DISTANT_NAMES):
        return "distant"
    if any(name in text for name in BOARD_NAMES):
        return "board"
    if any(name in text for name in PACKAGE_NAMES):
        return "package"
    return "other"


def discover_sensors():
    found = []
    thermal = Path("/sys/class/thermal")
    if thermal.is_dir():
        for zone in sorted(thermal.glob("thermal_zone*")):
            temp_path = zone / "temp"
            if not temp_path.exists():
                continue
            try:
                detail = (zone / "type").read_text(encoding="utf-8").strip()
            except OSError:
                detail = zone.name
            reading, _err = read_sysfs_temp(temp_path)
            kind = "unreadable" if reading is None else kind_of(detail)
            try:
                resolved = temp_path.resolve()
            except OSError:
                resolved = temp_path
            found.append(Sensor(temp_path, kind, reading, detail or zone.name, resolved))
    hwmon = Path("/sys/class/hwmon")
    if hwmon.is_dir():
        for node in sorted(hwmon.glob("hwmon*")):
            try:
                name = (node / "name").read_text(encoding="utf-8").strip()
            except OSError:
                name = node.name
            for temp_path in sorted(node.glob("temp*_input")):
                label_path = Path(str(temp_path).replace("_input", "_label"))
                label = ""
                if label_path.exists():
                    try:
                        label = label_path.read_text(encoding="utf-8").strip()
                    except OSError:
                        label = ""
                detail = name if not label else f"{name} {label}"
                reading, _err = read_sysfs_temp(temp_path)
                kind = "unreadable" if reading is None else kind_of(f"{name} {label}")
                try:
                    resolved = temp_path.resolve()
                except OSError:
                    resolved = temp_path
                found.append(Sensor(temp_path, kind, reading, detail, resolved))
    rank = {"board": 0, "package": 1, "other": 2, "distant": 3, "unreadable": 4}
    found.sort(key=lambda sensor: (rank.get(sensor.kind, 9), str(sensor.path)))
    return found


def same_sensor(left, right):
    if left is None or right is None:
        return False
    lp, rp = Path(left), Path(right)
    if lp == rp:
        return True
    try:
        return lp.resolve() == rp.resolve()
    except OSError:
        return False


def print_sensors(args):
    directive, note = active_directive(args.chrony_conf)
    sensors = discover_sensors()
    print("chrony temperature sensors")
    if directive is not None:
        print(f"configured: {directive.raw}")
        if getattr(directive, "error", None):
            print(f"points file: {directive.error}")
    elif note and "not readable" in str(note):
        print(f"chrony.conf: {note}")
    else:
        print("configured: no tempcomp line")
    if note and directive is not None:
        print(f"note: {note}")
    print()
    if not sensors:
        print("no thermal_zone or hwmon temperature files found")
        return 1
    resolved_owner = {}
    for sensor in sensors:
        resolved_owner.setdefault(sensor.resolved, sensor.path)
    for sensor in sensors:
        if sensor.reading is None:
            shown = "n/a"
        else:
            shown = fmt_temp(sensor.reading)
        mark = ""
        if directive is not None and same_sensor(sensor.path, directive.sensor):
            mark = "  configured"
        print(f"{sensor.path}{mark}")
        print(f"  {shown}  {sensor.kind}  {sensor.detail}")
        print(f"  {KIND_NOTE[sensor.kind]}")
        owner = resolved_owner.get(sensor.resolved)
        if owner is not None and owner != sensor.path:
            print(f"  same reading as {owner}")
        print()
    candidates = [
        sensor
        for sensor in sensors
        if sensor.reading is not None and sensor.kind in ("board", "package")
    ]
    if not candidates:
        candidates = [
            sensor
            for sensor in sensors
            if sensor.reading is not None and sensor.kind == "other"
        ]
    if candidates:
        ordered = sorted(
            candidates,
            key=lambda sensor: not (
                directive is not None and same_sensor(sensor.path, directive.sensor)
            ),
        )
        print("try first:")
        for sensor in ordered[:6]:
            print(f"  {sensor.path}")
            print(f"    {sensor.detail}")
        print()
        print(
            "Collect a first set while the machine temperature actually changes:"
        )
        print(
            f"  ./chrony-temp.py --collect 7200 --sensor {ordered[0].path}"
        )
    print(
        "hwmonN and thermal_zoneN numbers can change after a reboot. "
        "Recheck the path before adding it to chrony.conf."
    )
    print(
        "After the tempcomp line is installed, add tempcomp to the log directive."
    )
    print("Later runs with no --collect then refit from tempcomp.log.")
    return 0


def parse_chronyc_tracking(text):
    freq = skew = offset = None
    ref = ""
    leap = "?"
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip().lower()
        value = value.strip()
        if key == "reference id":
            if "(" in value and value.endswith(")"):
                ref = value[value.rfind("(") + 1:-1].strip()
            else:
                ref = value.split()[0] if value else ""
        elif key == "frequency":
            parts = value.split()
            if len(parts) >= 3 and parts[1].lower() == "ppm":
                freq = float(parts[0])
                if parts[2].lower() == "slow":
                    freq = -freq
        elif key == "skew":
            parts = value.split()
            skew = float(parts[0])
        elif key == "system time":
            parts = value.split()
            if len(parts) >= 3 and parts[1].lower() == "seconds":
                offset = float(parts[0])
                if parts[2].lower() == "slow":
                    offset = -offset
        elif key == "leap status":
            word = value.lower()
            if "not" in word:
                leap = "?"
            elif "insert" in word or word.startswith("+"):
                leap = "+"
            elif "delete" in word or word.startswith("-"):
                leap = "-"
            else:
                leap = "N"
    if freq is None or skew is None or offset is None:
        return None
    return freq, skew, offset, ref or "0.0.0.0", leap


def read_chronyc():
    try:
        proc = subprocess.run(
            ["chronyc", "-n", "tracking"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc)
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout).strip()
        return None, err or f"chronyc exited {proc.returncode}"
    parsed = parse_chronyc_tracking(proc.stdout)
    if parsed is None:
        return None, "chronyc tracking did not include frequency, skew, and system time"
    return parsed, None


def samples_header_sensor(path):
    if not path.exists():
        return None
    try:
        first = path.read_text(encoding="utf-8", errors="replace").splitlines()[:1]
    except OSError as exc:
        sys.exit(f"cannot read {path}: {exc}")
    if not first:
        return None
    line = first[0].strip()
    prefix = "# sensor "
    if line.startswith(prefix):
        return line[len(prefix):].strip()
    return None


def append_sample(path, sensor, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", encoding="utf-8", newline="") as handle:
        if new_file:
            handle.write(f"# sensor {sensor}\n")
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "time_utc",
                    "temp_mC",
                    "freq_ppm",
                    "skew_ppm",
                    "offset_s",
                    "ref",
                    "leap",
                    "comp_ppm",
                ]
            )
        else:
            writer = csv.writer(handle)
        writer.writerow(row)


def load_preliminary(path):
    readings = []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        sys.exit(f"cannot read {path}: {exc}")
    sensor = None
    for line in lines:
        if line.startswith("# sensor "):
            sensor = line.split(" ", 2)[2].strip()
            continue
        if not line.strip() or line.startswith("#") or line.startswith("time_utc"):
            continue
        parts = next(csv.reader([line]))
        if len(parts) < 8:
            continue
        try:
            stamp = datetime.strptime(parts[0], "%Y-%m-%dT%H:%M:%SZ").replace(
                tzinfo=timezone.utc
            )
            readings.append(
                Reading(
                    stamp,
                    float(parts[1]),
                    float(parts[7]),
                    float(parts[2]),
                    float(parts[3]),
                    float(parts[4]),
                    parts[5],
                    parts[6],
                )
            )
        except ValueError:
            continue
    readings.sort(key=lambda reading: reading.t)
    return sensor, readings


def collect(args, directive):
    sensor = args.sensor
    if sensor is None:
        sys.exit("--collect needs --sensor")
    reading, err = read_sysfs_temp(sensor)
    if reading is None:
        sys.exit(f"cannot read {sensor}: {err}")
    existing = samples_header_sensor(args.samples)
    if existing and not same_sensor(existing, sensor):
        sys.exit(
            f"{args.samples} already holds samples from {existing}. "
            f"Move that file aside before collecting {sensor}."
        )
    comp_sensor = None
    if directive is not None:
        comp_sensor = directive.sensor
        if getattr(directive, "error", None):
            sys.exit(
                f"chrony is using {directive.points_path}, which this script "
                f"cannot read ({directive.error}). The live frequency already "
                "includes that compensation."
            )
        probe, probe_err = read_sysfs_temp(comp_sensor)
        if probe is None:
            sys.exit(
                f"chrony tempcomp reads {comp_sensor}, which this script cannot "
                f"read ({probe_err}). Live samples would mix that compensation "
                "into the new fit."
            )
    print(
        f"collecting {fmt_dur(args.collect)} from {sensor} "
        f"every {fmt_interval(args.sample_interval)} s -> {args.samples}",
        file=sys.stderr,
    )
    if directive is None:
        print("no active tempcomp line; samples are the uncompensated clock", file=sys.stderr)
    else:
        print(
            f"removing compensation from {comp_sensor} before the sample is stored",
            file=sys.stderr,
        )
    deadline = time.monotonic() + args.collect
    next_tick = time.monotonic()
    taken = 0
    try:
        while True:
            now = datetime.now(timezone.utc).replace(microsecond=0)
            temp, temp_err = read_sysfs_temp(sensor)
            tracked, track_err = read_chronyc()
            comp = 0.0
            if directive is not None and temp is not None:
                basis, basis_err = read_sysfs_temp(comp_sensor)
                if basis is None:
                    temp_err = basis_err
                    temp = None
                else:
                    comp = directive_comp(directive, basis)
            if temp is None or tracked is None:
                print(
                    f"  skipped {fmt_time(now)}: {temp_err or track_err}",
                    file=sys.stderr,
                )
            else:
                freq, skew, offset, ref, leap = tracked
                append_sample(
                    args.samples,
                    sensor,
                    [
                        now.strftime("%Y-%m-%dT%H:%M:%SZ"),
                        f"{temp:.4f}",
                        f"{freq:.6f}",
                        f"{skew:.6f}",
                        f"{offset:.9f}",
                        ref,
                        leap,
                        f"{comp:.6f}",
                    ],
                )
                taken += 1
                print(
                    f"  {taken:4d}  {fmt_time(now)}  {fmt_temp(temp)}  "
                    f"freq {freq:+.3f} ppm  comp {comp:+.3f} ppm",
                    file=sys.stderr,
                )
            if time.monotonic() >= deadline:
                break
            next_tick += args.sample_interval
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        print("\nstopped early", file=sys.stderr)
    print(f"stored {taken} samples in {args.samples}", file=sys.stderr)
    return taken


def grid_temps(tmin, tmax, step, margin_mC):
    start = math.floor((tmin - margin_mC) / step) * step
    end = math.ceil((tmax + margin_mC) / step) * step
    temps = []
    temp = start
    while temp <= end + step * 0.01:
        temps.append(int(round(temp)))
        temp += step
    unique = []
    for temp in temps:
        if not unique or unique[-1] != temp:
            unique.append(temp)
    return unique


def build_points(temps, targets, coeff, step, margin_c, min_bin):
    t0, k0, k1, k2 = coeff
    tmin = min(temps)
    tmax = max(temps)
    margin_mC = margin_c * 1000.0
    grid = grid_temps(tmin, tmax, step, margin_mC)
    if len(grid) < 2:
        grid = [int(round(tmin)), int(round(tmax))]
        if grid[0] == grid[1]:
            grid[1] += step
    groups = {}
    for temp, target in zip(temps, targets):
        snapped = min(grid, key=lambda item: abs(item - temp))
        if abs(snapped - temp) > step / 2.0:
            continue
        if snapped < tmin or snapped > tmax:
            continue
        groups.setdefault(snapped, []).append(target)
    points = []
    from_bin = 0
    for temp in grid:
        bucket = groups.get(temp) or []
        inside = tmin <= temp <= tmax
        if inside and len(bucket) >= min_bin:
            comp = float(statistics.median(bucket))
            from_bin += 1
        else:
            comp = poly_at(temp, t0, k0, k1, k2)
        points.append((temp, comp))
    return points, from_bin


def write_points(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    text = "".join(f"{temp:d} {fmt_coeff(comp)}\n" for temp, comp in points)
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(math.fsum(value * value for value in values) / len(values))


def select_readings(readings, gaps, tracks, args):
    drops = empty_drops()
    drops["gap"] = len(gaps)
    windows = startup_windows(tracks, args.startup)
    kept = []
    for reading in readings:
        reason = classify(reading, windows, args)
        if reason == "ok":
            kept.append(reading)
        else:
            drops[reason] += 1
    return kept, drops, windows


def analyze(readings, args):
    """Fit the temperature shape of the clock rate.

    Tracking frequency is positive when the local clock runs fast. Tempcomp
    is positive when it speeds the clock up. The target compensation is the
    negation of the rate after the average rate is removed, and after any
    compensation that was already applied is added back.
    """
    temps = [reading.temp for reading in readings]
    uncomp = [reading.freq - reading.comp for reading in readings]
    mean_freq = statistics.fmean(uncomp)
    span_c = (max(temps) - min(temps)) / 1000.0
    info = {
        "n": len(readings),
        "mean_freq": mean_freq,
        "span_c": span_c,
        "tmin": min(temps),
        "tmax": max(temps),
        "t_start": readings[0].t,
        "t_end": readings[-1].t,
        "outliers": 0,
    }
    if len(readings) < MIN_FIT or span_c < MIN_TEMP_PREVIEW:
        info["status"] = "short"
        return info

    def fit_subset(idxs):
        subset_t = [temps[i] for i in idxs]
        # Re-center so a few dropped points do not leave a bias in the mean.
        subset_u = [uncomp[i] for i in idxs]
        mean = statistics.fmean(subset_u)
        subset_y = [-(rate - mean) for rate in subset_u]
        coeff = fit_quadratic(subset_t, subset_y)
        pred = [poly_at(temp, *coeff) for temp in subset_t]
        resid = [y - p for y, p in zip(subset_y, pred)]
        return coeff, resid, mean, subset_t, subset_y

    idxs = list(range(len(readings)))
    try:
        coeff, resid, mean_freq, subset_t, subset_y = fit_subset(idxs)
    except ValueError:
        info["status"] = "short"
        return info
    scale = statistics.pstdev(resid) if len(resid) > 1 else 0.0
    if scale > 0:
        kept_idx = [i for i, err in zip(idxs, resid) if abs(err) <= 4.0 * scale]
        if MIN_FIT <= len(kept_idx) < len(idxs):
            info["outliers"] = len(idxs) - len(kept_idx)
            idxs = kept_idx
            try:
                coeff, resid, mean_freq, subset_t, subset_y = fit_subset(idxs)
            except ValueError:
                info["status"] = "short"
                return info
    t0, k0, k1, k2 = coeff
    curve = [poly_at(temp, t0, k0, k1, k2) for temp in subset_t]
    info.update(
        {
            "mean_freq": mean_freq,
            "t0": t0,
            "k0": k0,
            "k1": k1,
            "k2": k2,
            "rms_before": statistics.pstdev([uncomp[i] for i in idxs]) if len(idxs) > 1 else 0.0,
            "rms_after": rms(resid),
            "ptp": max(curve) - min(curve) if curve else 0.0,
            "temps": subset_t,
            "targets": subset_y,
            "n_fit": len(subset_t),
            "limit": max(abs(value) for value in curve) > COMP_LIMIT if curve else False,
        }
    )
    return info


def status_of(info, args):
    if info.get("status") == "short" or "k0" not in info:
        return "short"
    span_s = (info["t_end"] - info["t_start"]).total_seconds()
    long_enough = (
        info["n_fit"] >= args.min_samples
        and span_s >= args.min_span
        and info["span_c"] >= args.min_temp_span
    )
    if not long_enough:
        return "preliminary"
    if info["ptp"] < FLAT_PPM:
        return "flat"
    if info.get("limit"):
        return "limit"
    return "ok"


def curve_diff(info, directive):
    if directive is None or "k0" not in info:
        return None
    if directive.points is None and directive.k0 is None:
        return None
    grid = grid_temps(info["tmin"], info["tmax"], 500, 0)
    if len(grid) < 2:
        grid = [info["tmin"], info["tmax"]]
    diffs = []
    for temp in grid:
        suggested = poly_at(temp, info["t0"], info["k0"], info["k1"], info["k2"])
        try:
            current = directive_comp(directive, temp)
        except (TypeError, ValueError):
            return None
        diffs.append(suggested - current)
    center = statistics.median(diffs)
    shape = [value - center for value in diffs]
    return {
        "constant": center,
        "max_abs": max(abs(value) for value in diffs),
        "shape": max(abs(value) for value in shape),
    }


def capture_span(readings):
    """Temperature and uncompensated clock-rate range of stored samples."""
    if not readings:
        return None
    temps = [reading.temp for reading in readings]
    rates = [reading.freq - reading.comp for reading in readings]
    return {
        "n": len(readings),
        "tmin": min(temps),
        "tmax": max(temps),
        "rmin": min(rates),
        "rmax": max(rates),
    }


def temp_line(sensor, interval, info):
    return (
        f"tempcomp {sensor} {fmt_interval(interval)} "
        f"{info['t0']:.0f} {fmt_coeff(info['k0'])} "
        f"{fmt_coeff(info['k1'])} {fmt_coeff(info['k2'])}"
    )


def print_report(context, info, drops, args, directive, points_path, points_meta):
    if context.get("count"):
        print()
        print(f"sensor {context['index']} of {context['count']}: {context['sensor']}")
    else:
        print("chrony temperature compensation")
        print(f"sensor: {context['sensor']}")
    print(f"sources: {context['source']}")
    if context.get("files"):
        print("files: " + ", ".join(context["files"]))
    if context.get("points_note"):
        print(
            f"points file on disk: {context['points_note']} "
            "(clock-rate samples are used instead)"
        )
    if info.get("t_start") is not None:
        print(f"span: {fmt_time(info['t_start'])} .. {fmt_time(info['t_end'])}")
    print(
        "filters: synchronised NTP, "
        f"|offset| < {args.max_offset:g} s, "
        f"skew < {args.max_skew:g} ppm, "
        f"warmup {args.startup:g} s after an unsynchronised sample, "
        f"tracking within {args.max_gap:g} s"
    )
    if not args.include_refclock:
        print("refclock selections are left out of the fit")
    print(
        "samples: "
        f"{context['temp_n']} temperature, "
        f"{context['paired']} paired, "
        f"{info.get('n_fit', info.get('n', 0))} used"
    )
    print(
        "left out: "
        f"gap {drops['gap']}, unsynced {drops['unsynced']}, "
        f"refclock {drops['refclock']}, warmup {drops['warmup']}, "
        f"offset {drops['offset']}, skew {drops['skew']}, "
        f"outlier {info.get('outliers', 0)}"
    )
    if "tmin" in info:
        print(
            f"temperature: {fmt_temp(info['tmin'])} .. {fmt_temp(info['tmax'])} "
            f"({info['span_c']:.1f} °C)"
        )
    status = info.get("status", "short")
    if context.get("max_comp", 0) > 1e-4:
        print(
            "the fit adds back compensation chrony had already applied, "
            f"up to {context['max_comp']:.3f} ppm"
        )
    if "k0" not in info:
        print(f"status: {status}")
        print(
            "no recommendation yet. The fit needs at least "
            f"{MIN_FIT} samples and {MIN_TEMP_PREVIEW:.1f} °C of temperature span."
        )
        _print_next(context, directive)
        _print_capture(context, info, args, directive, points_path)
        return
    print(
        f"clock rate: mean {info['mean_freq']:+.3f} ppm "
        "(positive means the uncompensated clock runs fast), "
        f"scatter {info['rms_before']:.3f} ppm"
    )
    print(
        f"fit rms {info['rms_after']:.3f} ppm, "
        f"peak-to-peak compensation {info['ptp']:.3f} ppm"
    )
    print(
        f"T0 {info['t0']:.0f} ({fmt_temp(info['t0'])}), "
        f"k0 {fmt_coeff(info['k0'])}, "
        f"k1 {fmt_coeff(info['k1'])} ppm per millidegree, "
        f"k2 {fmt_coeff(info['k2'])} ppm per millidegree squared"
    )
    for temp in (info["tmin"], info["t0"], info["tmax"]):
        comp = poly_at(temp, info["t0"], info["k0"], info["k1"], info["k2"])
        print(f"  at {fmt_temp(temp)}  comp {comp:+.3f} ppm")
    if directive is not None:
        print(f"configured: {directive.raw}")
        diff = curve_diff(info, directive)
        if diff is not None:
            print(
                f"  difference: max {diff['max_abs']:.3f} ppm, "
                f"of which {diff['constant']:+.3f} ppm is a constant shift "
                f"and {diff['shape']:.3f} ppm is shape"
            )
    print(f"status: {status}")
    for line in _notes(info, status, args, context):
        print(f"note: {line}")
    if status == "short":
        _print_next(context, directive)
        _print_capture(context, info, args, directive, points_path)
        return
    tag = "suggested" if status == "ok" else "preview"
    print()
    print(
        f"result status={status} source={context['label']} "
        f"samples={info['n_fit']} span={(info['t_end'] - info['t_start']).total_seconds():.0f}s "
        f"temp_span={info['span_c']:.2f}C "
        f"t0={info['t0']:.0f} k0={fmt_coeff(info['k0'])} "
        f"k1={fmt_coeff(info['k1'])} k2={fmt_coeff(info['k2'])} "
        f"ptp={info['ptp']:.3f} rms={info['rms_after']:.3f}"
    )
    print(f"{tag}: {temp_line(context['sensor'], context['interval'], info)}")
    if points_path is not None:
        from_bin, total = points_meta
        print(
            f"points: {points_path} ({total} points, "
            f"{total - from_bin} from the polynomial, {from_bin} from bin medians, "
            f"step {args.step} mC)"
        )
        print(
            f"{tag} points: tempcomp {context['sensor']} "
            f"{fmt_interval(context['interval'])} {points_path}"
        )
    _print_next(context, directive)
    _print_capture(context, info, args, directive, points_path)


def _print_capture(context, info, args, directive, points_path):
    """Closing summary for a finished --collect run."""
    if args.collect is None:
        return
    captured = context.get("captured")
    print()
    print("captured")
    if not captured:
        print(f"no samples were stored in {args.samples.name}")
        return
    span_c = (captured["tmax"] - captured["tmin"]) / 1000.0
    sweep = captured["rmax"] - captured["rmin"]
    print(
        f"{captured['n']} samples in {args.samples.name}: "
        f"temperature {fmt_temp(captured['tmin'])} .. {fmt_temp(captured['tmax'])} "
        f"({span_c:.1f} °C)"
    )
    print(
        f"clock rate {captured['rmin']:+.3f} .. {captured['rmax']:+.3f} ppm "
        f"(sweep {sweep:.3f} ppm, positive means the clock runs fast)"
    )
    print()
    if "k0" not in info:
        print(
            "No chrony.conf line yet. The fit needs at least "
            f"{MIN_FIT} samples and {MIN_TEMP_PREVIEW:.1f} °C of temperature span."
        )
        return
    status = info.get("status", "short")
    if status != "ok":
        print(
            f"Status is {status}, so treat this as a first estimate "
            "rather than a finished line."
        )
    same = directive is not None and same_sensor(context["sensor"], directive.sensor)
    if same:
        print("Consider replacing the active tempcomp line in chrony.conf with:")
    else:
        print("Consider adding this line to chrony.conf:")
    print()
    print(temp_line(context["sensor"], context["interval"], info))
    if points_path is not None:
        print()
        print("Or this line, which uses the points file:")
        print()
        print(
            f"tempcomp {context['sensor']} "
            f"{fmt_interval(context['interval'])} {points_path}"
        )


def _notes(info, status, args, context):
    notes = []
    if abs(info["k0"]) > 1.0:
        notes.append(
            f"k0 is {info['k0']:+.3f} ppm at T0. Chrony still tracks the average "
            "rate itself. A large k0 spends the ±10 ppm tempcomp budget."
        )
    if status == "flat":
        notes.append(
            f"The curve moves only {info['ptp']:.3f} ppm across this temperature "
            "range. Tempcomp will not change the clock until the range is wider."
        )
    if status == "limit":
        notes.append(
            "Part of this curve is outside ±10 ppm. Chrony ignores those readings."
        )
    if info.get("point_over") and status != "limit":
        notes.append(
            f"{info['point_over']} points outside the observed temperatures are "
            "beyond ±10 ppm. Chrony ignores the sensor at those temperatures."
        )
    if status == "preliminary":
        unmet = []
        span_s = (info["t_end"] - info["t_start"]).total_seconds()
        if info["n_fit"] < args.min_samples:
            unmet.append(f"{info['n_fit']} samples and the bar is {args.min_samples}")
        if span_s < args.min_span:
            unmet.append(
                f"the time span is {fmt_dur(span_s)} and the bar is {fmt_dur(args.min_span)}"
            )
        if info["span_c"] < args.min_temp_span:
            unmet.append(
                f"the temperature span is {info['span_c']:.1f} °C "
                f"and the bar is {args.min_temp_span:.1f} °C"
            )
        detail = "; ".join(unmet) if unmet else "the window is still short"
        notes.append(f"Preview only. {detail[0].upper() + detail[1:]}.")
    if context["label"] == "preliminary":
        notes.append(
            "These samples are the preliminary file. After tempcomp is in "
            "chrony.conf, add tempcomp to the log directive and rerun with no "
            "--collect. That refit uses chrony's own tempcomp.log."
        )
    if len(info["temps"]) >= 4:
        q = statistics.quantiles(info["temps"], n=4, method="inclusive")
        iqr = (q[2] - q[0]) / 1000.0
    else:
        iqr = info["span_c"]
    if info["span_c"] > 0 and iqr < 0.2 * info["span_c"]:
        notes.append(
            "Most samples sit in a narrow band. The curve outside that band "
            "rests on fewer readings."
        )
    return notes


def _print_next(context, directive):
    if context["label"] == "chrony":
        return
    if directive is None:
        print(
            "Next, install the tempcomp line and add tempcomp to the log directive:"
        )
        print("  log tracking tempcomp")
        return
    print("Next, add tempcomp to the log directive so later runs use tempcomp.log:")
    print("  log tracking tempcomp")


def prepare_preliminary(args, directive):
    if not args.samples.exists():
        return None, f"no preliminary file at {args.samples}"
    sensor, readings = load_preliminary(args.samples)
    if args.sensor and sensor and not same_sensor(args.sensor, sensor):
        return None, (
            f"{args.samples} belongs to {sensor}, not {args.sensor}"
        )
    sensor = sensor or args.sensor
    if sensor is None:
        return None, f"{args.samples} does not name a sensor"
    # Preliminary rows are already paired. Gaps stay empty.
    dummy_tracks = [
        Track(reading.t, reading.ref, reading.freq, reading.skew, reading.offset, reading.leap)
        for reading in readings
    ]
    kept, drops, _windows = select_readings(readings, [], dummy_tracks, args)
    context = {
        "captured": capture_span(readings),
        "source": f"preliminary {args.samples}",
        "label": "preliminary",
        "files": [args.samples.name],
        "sensor": sensor,
        "removed": None,
        "temp_n": len(readings),
        "paired": len(readings),
        "interval": args.interval or (directive.interval if directive else 60.0),
        "max_comp": max((abs(reading.comp) for reading in kept), default=0.0),
    }
    return (kept, drops, context), None


def emit(kept, drops, context, args, directive, advice=None):
    if not kept:
        info = {"status": "short", "n": 0, "outliers": 0}
        print_report(context, info, drops, args, directive, None, (0, 0))
        return 1
    kept.sort(key=lambda reading: reading.t)
    info = analyze(kept, args)
    info["status"] = status_of(info, args)
    drops["outlier"] = info.get("outliers", 0)
    points_path = None
    points_meta = (0, 0)
    if "k0" in info and not args.no_write:
        points, from_bin = build_points(
            info["temps"],
            info["targets"],
            (info["t0"], info["k0"], info["k1"], info["k2"]),
            args.step,
            args.margin,
            args.min_bin,
        )
        if args.points is not None and context.get("count", 1) == 1:
            points_path = Path(args.points).resolve()
        else:
            points_path = default_points_path(context["sensor"]).resolve()
        try:
            write_points(points_path, points)
        except OSError as exc:
            print(f"cannot write {points_path}: {exc}", file=sys.stderr)
            points_path = None
        else:
            points_meta = (from_bin, len(points))
            info["point_over"] = sum(1 for _temp, comp in points if abs(comp) > COMP_LIMIT)
    print_report(context, info, drops, args, directive, points_path, points_meta)
    if advice is not None:
        advice.extend(conf_lines_from(context, info, points_path))
    if args.strict and info["status"] != "ok":
        return 2
    if info["status"] == "short":
        return 1
    return 0


def conf_lines_from(context, info, points_path):
    if "k0" not in info or info.get("status") == "short":
        return []
    lines = [temp_line(context["sensor"], context["interval"], info)]
    if points_path is not None:
        lines.append(
            f"tempcomp {context['sensor']} "
            f"{fmt_interval(context['interval'])} {points_path}"
        )
    return lines


@dataclass
class SensorData:
    sensor: str
    csvs: list
    points: list
    use_logs: bool = False
    bundle: tuple | None = None


def interval_for(args, directive, sensor):
    if args.interval is not None:
        return args.interval
    if directive is not None and same_sensor(directive.sensor, sensor):
        return directive.interval
    return 60.0


def load_chrony(args):
    temp_paths = log_files(args.log_dir, "tempcomp", args.include_rotated)
    tr_paths = log_files(args.log_dir, "tracking", args.include_rotated)
    if not temp_paths:
        return None, "no tempcomp.log"
    if not tr_paths:
        return None, "no tracking.log"
    temps = load_tempcomp(temp_paths)
    tracks = load_tracking(tr_paths)
    paired, gaps = pair_readings(temps, tracks, args.max_gap)
    files = [path.name for path in temp_paths + tr_paths]
    return (temps, paired, gaps, tracks, files), None


def points_slug(path):
    name = path.name
    if name.endswith("-comp.points"):
        return name[: -len("-comp.points")]
    prefix = "chrony-tempcomp-"
    if name.startswith(prefix) and name.endswith(".points"):
        return name[len(prefix) : -len(".points")]
    return None


def local_points_files(args):
    found = {}
    for pattern in ("*-comp.points", "chrony-tempcomp-*.points"):
        for path in Path.cwd().glob(pattern):
            if path.is_file():
                found[path.resolve()] = path
    if args.points_explicit and args.points.is_file():
        found[args.points.resolve()] = args.points
    return sorted(found.values(), key=lambda path: path.name)


def local_sample_files(args):
    if args.source == "chrony":
        return []
    if args.samples_explicit:
        return [args.samples]
    return sorted(path for path in Path.cwd().glob("*-samples.csv") if path.is_file())


def resolve_slug(slug, groups, directive, args, sysfs):
    candidates = [group.sensor for group in groups]
    if directive is not None:
        candidates.append(directive.sensor)
    if args.sensor:
        candidates.append(args.sensor)
    for candidate in candidates:
        if sensor_slug(candidate) == slug:
            return candidate
    for sensor in sysfs:
        if sensor.reading is None:
            continue
        if sensor_slug(sensor.path) == slug:
            return str(sensor.path)
    return None


def group_for(groups, sensor):
    for group in groups:
        if same_sensor(group.sensor, sensor):
            return group
    group = SensorData(str(sensor), [], [], False, None)
    groups.append(group)
    return group


def discover_groups(args, directive):
    """Sensors that have logs, a csv, or a points file."""
    groups = []
    notes = []
    if args.samples_explicit and not args.samples.exists():
        sys.exit(f"no preliminary file at {args.samples}")
    for path in local_sample_files(args):
        header = samples_header_sensor(path)
        if not header:
            notes.append(f"{path.name} has no sensor header")
            continue
        if args.sensor and not same_sensor(header, args.sensor):
            if args.samples_explicit:
                sys.exit(f"{path} belongs to {header}, not {args.sensor}")
            continue
        group_for(groups, header).csvs.append(path)

    logs, log_err = (None, None)
    if args.source != "preliminary":
        logs, log_err = load_chrony(args)
    temps = logs[0] if logs else []
    if temps:
        if directive is not None:
            log_sensor = directive.sensor
        elif args.sensor:
            log_sensor = args.sensor
        else:
            log_sensor = None
        if log_sensor is None:
            notes.append(
                "tempcomp.log has readings, but no tempcomp line names their sensor. "
                "Pass --sensor to use them."
            )
        elif args.sensor is None or same_sensor(args.sensor, log_sensor):
            group = group_for(groups, log_sensor)
            group.use_logs = True
            group.bundle = logs
        else:
            notes.append(f"tempcomp.log follows {log_sensor}, not {args.sensor}")
    elif args.source == "chrony":
        notes.append(log_err or "tempcomp log has no temperature rows")

    if args.source == "auto":
        sysfs = None
        for path in local_points_files(args):
            slug = points_slug(path)
            if slug is None:
                notes.append(f"{path.name} is not a sensor points file")
                continue
            if sysfs is None:
                sysfs = discover_sensors()
            sensor = resolve_slug(slug, groups, directive, args, sysfs)
            if sensor is None:
                notes.append(f"{path.name} does not match a sensor path")
                continue
            if args.sensor and not same_sensor(sensor, args.sensor):
                continue
            group = group_for(groups, sensor)
            if path.resolve() not in {item.resolve() for item in group.points}:
                group.points.append(path)

    def sort_key(group):
        configured = directive is not None and same_sensor(group.sensor, directive.sensor)
        return (0 if configured else 1, group.sensor)

    groups.sort(key=sort_key)
    return groups, notes, log_err


def print_banner(count, sensors, rotated):
    print("chrony temperature compensation")
    noun = "sensor" if count == 1 else "sensors"
    print(f"analyzing {count} {noun}")
    for sensor in sensors:
        print(f"  {sensor}")
    if rotated is True:
        print("rotated logs are included")
    elif rotated is False:
        print("rotated logs are skipped")


def choose_points_file(paths):
    def rank(path):
        prefer = 0 if path.name.endswith("-comp.points") else 1
        try:
            age = -path.stat().st_mtime
        except OSError:
            age = 0
        return (prefer, age, path.name)

    return sorted(paths, key=rank)[0]


def report_points_only(group, index, count, args, directive, advice):
    path = choose_points_file(group.points)
    points, err = load_points(path)
    print()
    print(f"sensor {index} of {count}: {group.sensor}")
    names = ", ".join(item.name for item in group.points)
    print(f"sources: points file {names}")
    if err:
        print(f"could not read {path}: {err}")
        return 1
    temps = [temp for temp, _comp in points]
    span_c = (max(temps) - min(temps)) / 1000.0
    print(
        f"temperature: {fmt_temp(min(temps))} .. {fmt_temp(max(temps))} "
        f"({span_c:.1f} °C, {len(points)} points)"
    )
    print("No clock-rate samples for this sensor. The line uses this points file.")
    interval = interval_for(args, directive, group.sensor)
    line = f"tempcomp {group.sensor} {fmt_interval(interval)} {path.resolve()}"
    print()
    print("Consider adding this line to chrony.conf:")
    print()
    print(line)
    advice.append(line)
    try:
        coeff = fit_quadratic(temps, [comp for _temp, comp in points])
    except ValueError:
        return 0
    info = {"t0": coeff[0], "k0": coeff[1], "k1": coeff[2], "k2": coeff[3], "status": "points"}
    quad = temp_line(group.sensor, interval, info)
    print()
    print("Quadratic form of that curve:")
    print()
    print(quad)
    advice.append(quad)
    return 0


def measurement_context(args, directive, group, index, count):
    readings = []
    files = []
    gaps = []
    tracks = None
    log_temps = 0
    source_bits = []
    if group.use_logs and group.bundle is not None:
        temps, paired, log_gaps, tracks, log_files = group.bundle
        readings.extend(paired)
        gaps.extend(log_gaps)
        files.extend(log_files)
        log_temps = len(temps)
        source_bits.append("chrony logs")
    seen = {reading.t for reading in readings}
    csv_n = 0
    for path in group.csvs:
        _sensor, rows = load_preliminary(path)
        added = 0
        for row in rows:
            if row.t in seen:
                continue
            readings.append(row)
            seen.add(row.t)
            added += 1
        csv_n += added
        files.append(path.name)
    if group.csvs:
        source_bits.append("csv")
    readings.sort(key=lambda reading: reading.t)
    if tracks is None:
        tracks = [
            Track(
                reading.t,
                reading.ref,
                reading.freq,
                reading.skew,
                reading.offset,
                reading.leap,
            )
            for reading in readings
        ]
    kept, drops, _windows = select_readings(readings, gaps, tracks, args)
    context = {
        "index": index,
        "count": count,
        "source": " and ".join(source_bits) if source_bits else "none",
        "label": "chrony" if group.use_logs else "preliminary",
        "files": files,
        "sensor": group.sensor,
        "temp_n": log_temps + csv_n,
        "paired": len(readings),
        "interval": interval_for(args, directive, group.sensor),
        "max_comp": max((abs(reading.comp) for reading in kept), default=0.0),
        "points_note": ", ".join(path.name for path in group.points),
    }
    return kept, drops, context


def print_conf_lines(lines):
    if not lines:
        return
    print()
    print("chrony.conf")
    for line in lines:
        print(line)


def run(args):
    args.samples_explicit = args.samples is not None
    args.points_explicit = args.points is not None
    directive, note = active_directive(args.chrony_conf)
    if note and directive is not None:
        print(f"note: {note}", file=sys.stderr)
    if args.collect is not None:
        resolve_output_paths(args, directive)
        collect(args, directive)
        prepared, err = prepare_preliminary(args, directive)
        if err:
            sys.exit(err)
        kept, drops, context = prepared
        context["index"] = 1
        context["count"] = 1
        print_banner(1, [context["sensor"]], None)
        return emit(kept, drops, context, args, directive)

    groups, notes, log_err = discover_groups(args, directive)
    if not groups:
        print("chrony temperature compensation")
        print("analyzing 0 sensors")
        for line in notes:
            print(line)
        if log_err and args.source != "chrony":
            print(f"chrony log: {log_err}")
        print("No sensor has a tempcomp log, a *-samples.csv file, or a points file.")
        print("Collect a first set while the temperature changes:")
        print("  ./chrony-temp.py --sensors")
        print("  ./chrony-temp.py --collect 7200 --sensor SENSOR")
        return 1
    if args.points_explicit and len(groups) > 1:
        print(
            "--points is one file, so each sensor keeps its own points file",
            file=sys.stderr,
        )
    print_banner(
        len(groups),
        [group.sensor for group in groups],
        args.include_rotated if args.source != "preliminary" else None,
    )
    for line in notes:
        print(line)
    advice = []
    codes = []
    for index, group in enumerate(groups, start=1):
        if not group.use_logs and not group.csvs and group.points:
            code = report_points_only(group, index, len(groups), args, directive, advice)
            codes.append(code)
            continue
        if not group.use_logs and not group.csvs:
            print()
            print(f"sensor {index} of {len(groups)}: {group.sensor}")
            print("no samples for this sensor")
            codes.append(1)
            continue
        kept, drops, context = measurement_context(
            args, directive, group, index, len(groups)
        )
        code = emit(kept, drops, context, args, directive, advice)
        codes.append(code)
    print_conf_lines(advice)
    if args.strict and any(code == 2 for code in codes):
        return 2
    if advice:
        return 0
    return 1


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Recommend a chrony tempcomp line from temperature and clock rate.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  ./chrony-temp.py --sensors\n"
            "  ./chrony-temp.py --collect 7200 --sensor /sys/class/thermal/thermal_zone0/temp\n"
            "  ./chrony-temp.py\n"
            "  ./chrony-temp.py --sensor /sys/class/thermal/thermal_zone0/temp\n"
            "  ./chrony-temp.py --no-include-rotated\n"
            "\n"
            "With no flags, every sensor that has data is reported. Data is\n"
            "tempcomp logs (rotated logs included), *-samples.csv, and points\n"
            "files in the working directory. --sensor limits the run to one.\n"
        ),
    )
    parser.add_argument(
        "--sensors",
        action="store_true",
        help="list sysfs temperature sensors and which ones are worth trying",
    )
    parser.add_argument(
        "--collect",
        type=float,
        default=None,
        metavar="SECONDS",
        help="sample the sensor and chronyc tracking for this many seconds, then fit",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=30.0,
        help="seconds between preliminary samples (default 30)",
    )
    parser.add_argument(
        "--sensor",
        help="temperature file to collect, or the only sensor to fit",
    )
    parser.add_argument(
        "--source",
        choices=("auto", "chrony", "preliminary"),
        default="auto",
        help="auto uses chrony logs, csv files, and points files for every sensor (default auto)",
    )
    parser.add_argument(
        "--samples",
        type=Path,
        default=None,
        help=(
            "preliminary sample file "
            "(default ./<sensor>-samples.csv, such as thermal_zone0-temp-samples.csv)"
        ),
    )
    parser.add_argument(
        "--points",
        type=Path,
        default=None,
        help=(
            "points file to write "
            "(default ./<sensor>-comp.points, such as thermal_zone0-temp-comp.points)"
        ),
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="print the suggestion without writing the points file",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("/var/log/chrony"),
        help="directory with tempcomp.log and tracking.log (default /var/log/chrony)",
    )
    parser.add_argument(
        "--chrony-conf",
        type=Path,
        default=Path("/etc/chrony/chrony.conf"),
        help="config used to find the active tempcomp line (default /etc/chrony/chrony.conf)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=None,
        help="update interval in the suggested line (default: the configured interval, else 60)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=250,
        help="points-file temperature step in millidegrees (default 250)",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=2.0,
        help="degrees to extend the points file past the observed range (default 2)",
    )
    parser.add_argument(
        "--min-bin",
        type=int,
        default=8,
        help="samples in a temperature step before that point follows the data instead of the polynomial (default 8)",
    )
    parser.add_argument(
        "--include-rotated",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="read tempcomp.log.N and tracking.log.N (default on)",
    )
    parser.add_argument(
        "--include-refclock",
        action="store_true",
        help="also use tracking samples whose selected source is a refclock",
    )
    parser.add_argument(
        "--startup",
        type=float,
        default=900.0,
        help="seconds to skip after an unsynchronised tracking sample (default 900)",
    )
    parser.add_argument(
        "--max-offset",
        type=float,
        default=0.01,
        help="skip samples while |tracking offset| is at least this many seconds (default 0.01)",
    )
    parser.add_argument(
        "--max-skew",
        type=float,
        default=0.5,
        help="skip samples whose frequency skew is at least this many ppm (default 0.5)",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=900.0,
        help="skip a temperature reading with no tracking sample within this many seconds (default 900)",
    )
    parser.add_argument(
        "--min-span",
        type=float,
        default=7200.0,
        help="seconds of samples required before status is ok (default 7200)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="samples required before status is ok (default 20)",
    )
    parser.add_argument(
        "--min-temp-span",
        type=float,
        default=2.0,
        help="degrees of temperature span required before status is ok (default 2)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit 2 unless status is ok",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.sensors and args.collect is not None:
        sys.exit("use either --sensors or --collect")
    if args.step < 1:
        sys.exit("step must be at least 1 millidegree")
    if args.margin < 0 or args.min_bin < 1:
        sys.exit("margin must be >= 0 and min-bin must be >= 1")
    if args.collect is not None and args.collect <= 0:
        sys.exit("collect duration must be positive")
    if args.sample_interval <= 0:
        sys.exit("sample interval must be positive")
    if args.startup < 0 or args.max_offset < 0 or args.max_skew < 0 or args.max_gap < 0:
        sys.exit("startup, max-offset, max-skew, and max-gap must be >= 0")
    if args.min_span < 0 or args.min_samples < 1 or args.min_temp_span < 0:
        sys.exit("the sample minimums must be non-negative, and min-samples at least 1")
    if args.interval is not None and args.interval <= 0:
        sys.exit("interval must be positive")
    if args.sensors:
        return print_sensors(args)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
