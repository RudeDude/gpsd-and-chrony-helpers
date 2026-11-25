#!/usr/bin/env python3
"""Estimate system-clock holdover error from chrony.

Holdover starts when every NTP server and refclock has stopped. chronyd
keeps the last frequency. A tempcomp line keeps reading its sensor,
because that does not use those sources.

Between clock updates chrony grows its root dispersion by

    elapsed * (skew + |residual frequency| + maxclockerror)

The error bound is root delay / 2 + that dispersion. maxclockerror
defaults to 1 ppm. It is an allowance for an undisciplined clock, not a
measurement. The skew is the measured uncertainty of the current fit.

tracking.log adds a second figure. While NTP was selected, the frequency
column is the correction chrony was still making. In holdover those
corrections stop, so a frequency change of that size becomes time error.

The temperature table assumes the tempcomp line stays on. A sensor move
changes the compensation. The 1 day and 7 day columns are that walk of
the clock, which is the crystal drift the model is removing.
"""

import argparse
import ipaddress
import re
import statistics
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

DATE_RE = re.compile(r"\d{4}-\d{2}-\d{2}")
DEFAULT_MAXCLOCKERROR = 1.0
HORIZON_UNIT = {"s": 1.0, "m": 60.0, "h": 3600.0, "d": 86400.0}
F_TO_C = 5.0 / 9.0
DAY = 86400.0
WEEK = 7.0 * DAY


@dataclass
class Track:
    t: datetime
    ref: str
    freq: float
    skew: float
    offset: float
    leap: str
    root_delay: float
    root_disp: float
    max_error: float


@dataclass
class Directive:
    sensor: str
    interval: float
    t0: float | None = None
    k0: float | None = None
    k1: float | None = None
    k2: float | None = None
    points: list | None = None
    raw: str = ""


@dataclass
class Live:
    ref: str
    stratum: int | None
    offset: float
    frequency: float
    residual: float
    skew: float
    root_delay: float
    root_disp: float
    update_interval: float | None
    synced: bool
    when: datetime | None


def parse_stamp(date_s, time_s):
    text = f"{date_s} {time_s}"
    if "." in time_s:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S.%f")
    else:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(tzinfo=timezone.utc)


def fmt_time(stamp):
    return stamp.strftime("%Y-%m-%d %H:%M:%SZ")


def fmt_dur(seconds):
    seconds = int(round(seconds))
    sign = "-" if seconds < 0 else ""
    seconds = abs(seconds)
    days, rem = divmod(seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, secs = divmod(rem, 60)
    if days:
        return f"{sign}{days}d {hours}h"
    if hours:
        return f"{sign}{hours}h {minutes:02d}m"
    if minutes:
        return f"{sign}{minutes}m {secs:02d}s"
    return f"{sign}{secs}s"


def fmt_span(seconds):
    """Compact label for a horizon."""
    if abs(seconds - 86400) < 1:
        return "1d"
    if abs(seconds - 7 * 86400) < 1:
        return "7d"
    if seconds % 3600 == 0 and seconds >= 3600:
        return f"{int(seconds // 3600)}h"
    if seconds % 60 == 0 and seconds >= 60:
        return f"{int(seconds // 60)}m"
    return f"{int(seconds)}s"


def fmt_abs(seconds):
    value = abs(seconds)
    if value < 1e-6:
        return f"{value * 1e9:.1f} ns"
    if value < 1e-3:
        return f"{value * 1e6:.1f} us"
    if value < 1:
        return f"{value * 1e3:.2f} ms"
    if value < 60:
        return f"{value:.2f} s"
    return f"{value / 60:.1f} min"


def fmt_signed(seconds):
    if abs(seconds) < 5e-9:
        return "0"
    side = "slow" if seconds < 0 else "fast"
    return f"{fmt_abs(seconds)} {side}"


def ppm_to_seconds(ppm, elapsed):
    return ppm * 1e-6 * elapsed


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


def is_ntp(ref):
    try:
        return not ipaddress.ip_address(ref).is_unspecified
    except ValueError:
        return False


def is_unspecified(ref):
    try:
        return ipaddress.ip_address(ref).is_unspecified
    except ValueError:
        return False


def parse_rate(value):
    """Signed ppm. '16.667 ppm slow' is negative. '-0.002 ppm' is already signed."""
    parts = value.split()
    if not parts:
        raise ValueError(value)
    number = float(parts[0])
    if len(parts) >= 3 and parts[1].lower() == "ppm" and parts[2].lower() == "slow":
        return -abs(number)
    if len(parts) >= 3 and parts[1].lower() == "ppm" and parts[2].lower() == "fast":
        return abs(number)
    return number


def parse_offset(value):
    """Signed seconds. 'slow of NTP' means the clock is behind."""
    parts = value.split()
    if len(parts) < 2:
        raise ValueError(value)
    number = float(parts[0])
    if "slow" in value.lower():
        return -abs(number)
    if "fast" in value.lower():
        return abs(number)
    return number


def parse_chronyc(text, when):
    fields = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        fields[key.strip().lower()] = value.strip()
    needed = (
        "reference id",
        "system time",
        "frequency",
        "residual freq",
        "skew",
        "root delay",
        "root dispersion",
    )
    missing = [key for key in needed if key not in fields]
    if missing:
        return None, "chronyc tracking is missing " + ", ".join(missing)
    ref = fields["reference id"]
    if "(" in ref and ref.endswith(")"):
        ref = ref[ref.rfind("(") + 1:-1].strip()
    elif ref:
        ref = ref.split()[0]
    stratum = None
    if "stratum" in fields:
        try:
            stratum = int(fields["stratum"].split()[0])
        except ValueError:
            stratum = None
    update = None
    if "update interval" in fields:
        try:
            update = float(fields["update interval"].split()[0])
        except ValueError:
            update = None
    leap = fields.get("leap status", "").lower()
    synced = "not" not in leap
    try:
        live = Live(
            ref or "0.0.0.0",
            stratum,
            parse_offset(fields["system time"]),
            parse_rate(fields["frequency"]),
            parse_rate(fields["residual freq"]),
            float(fields["skew"].split()[0]),
            float(fields["root delay"].split()[0]),
            float(fields["root dispersion"].split()[0]),
            update,
            synced,
            when,
        )
    except ValueError as exc:
        return None, f"chronyc tracking: {exc}"
    return live, None


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
    return parse_chronyc(proc.stdout, datetime.now(timezone.utc).replace(microsecond=0))


def load_tracking(paths):
    rows = []
    for path in paths:
        for line in read_lines(path):
            parts = line.split()
            if len(parts) < 14 or not DATE_RE.fullmatch(parts[0]):
                continue
            try:
                rows.append(
                    Track(
                        parse_stamp(parts[0], parts[1]),
                        parts[2],
                        float(parts[4]),
                        float(parts[5]),
                        float(parts[6]),
                        parts[7],
                        float(parts[11]),
                        float(parts[12]),
                        float(parts[13]),
                    )
                )
            except ValueError:
                continue
    rows.sort(key=lambda row: row.t)
    return rows


def startup_windows(rows, startup_s):
    windows = []
    if startup_s <= 0:
        return windows
    for row in rows:
        if is_unspecified(row.ref) or row.leap == "?":
            windows.append((row.t, row.t + timedelta(seconds=startup_s)))
    return windows


def in_windows(stamp, windows):
    return any(start <= stamp < end for start, end in windows)


def live_from_log(row):
    return Live(
        row.ref,
        None,
        row.offset,
        row.freq,
        0.0,
        row.skew,
        row.root_delay,
        row.root_disp,
        None,
        row.leap != "?" and not is_unspecified(row.ref),
        row.t,
    )


def load_maxclockerror(path):
    if path is None or not path.is_file():
        return DEFAULT_MAXCLOCKERROR, "default"
    value = None
    for line in read_lines(path):
        body = line.split("#", 1)[0].strip().split()
        if len(body) >= 2 and body[0] == "maxclockerror":
            try:
                value = float(body[1])
            except ValueError:
                continue
    if value is None:
        return DEFAULT_MAXCLOCKERROR, "default"
    return value, str(path)


def load_points(path):
    points = []
    try:
        lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
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
    points.sort()
    if len(points) < 2:
        return None
    return points


def parse_directive(line):
    body = line.split("#", 1)[0].strip()
    parts = body.split()
    if len(parts) < 4 or parts[0] != "tempcomp":
        return None
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
        return Directive(parts[1], interval, t0, k0, k1, k2, raw=raw)
    return Directive(parts[1], interval, points=load_points(rest[0]), raw=raw)


def load_directive(path):
    if path is None or not path.is_file():
        return None
    found = []
    for line in read_lines(path):
        directive = parse_directive(line)
        if directive is not None:
            found.append(directive)
    if not found:
        return None
    return found[-1]


def poly_at(temp, directive):
    if directive.points:
        points = directive.points
        upper = 1
        while upper + 1 < len(points) and points[upper][0] < temp:
            upper += 1
        t1, c1 = points[upper - 1]
        t2, c2 = points[upper]
        if t2 == t1:
            return c1
        return (temp - t1) / (t2 - t1) * (c2 - c1) + c1
    delta = temp - directive.t0
    return directive.k0 + delta * directive.k1 + delta * delta * directive.k2


def read_sensor(path):
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


def display_temp(raw):
    """Linux sysfs temperatures are millidegrees. Small numbers are already °C."""
    if abs(raw) >= 200:
        return raw / 1000.0
    return raw


def slope_ppm_per_c(directive, temp):
    """Derivative of compensation, in ppm per °C, at this raw sensor reading."""
    if directive is None or directive.k1 is None:
        return None
    # k1 is ppm per sensor unit. Millidegree files need a factor of 1000.
    per_unit = directive.k1 + 2.0 * directive.k2 * (temp - directive.t0)
    if abs(temp) >= 200:
        return per_unit * 1000.0
    return per_unit


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
    rows.sort()
    return rows


def usable(row, windows):
    if row.leap == "?" or not is_ntp(row.ref):
        return False
    if in_windows(row.t, windows):
        return False
    return True


def segments(rows, windows, max_gap):
    groups = []
    current = []
    for row in rows:
        if not usable(row, windows):
            if current:
                groups.append(current)
                current = []
            continue
        if current and (row.t - current[-1].t).total_seconds() > max_gap:
            groups.append(current)
            current = []
        current.append(row)
    if current:
        groups.append(current)
    return groups


def frequency_moves(groups, horizon):
    """|frequency change| for pairs about one horizon apart, in ppm."""
    moves = []
    limit = horizon * 1.5
    for group in groups:
        later = 0
        for sample in group:
            target = sample.t + timedelta(seconds=horizon)
            while later < len(group) and group[later].t < target:
                later += 1
            if later >= len(group):
                break
            elapsed = (group[later].t - sample.t).total_seconds()
            if elapsed > limit:
                continue
            moves.append(abs(group[later].freq - sample.freq))
    return moves


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * fraction
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    weight = pos - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def parse_horizons(text):
    horizons = []
    for piece in text.split(","):
        item = piece.strip().lower()
        if not item:
            continue
        try:
            if item[-1] in HORIZON_UNIT and not item[-1].isdigit():
                horizons.append(float(item[:-1]) * HORIZON_UNIT[item[-1]])
            else:
                horizons.append(float(item))
        except ValueError:
            sys.exit(f"bad horizon {piece!r}")
        if horizons[-1] <= 0:
            sys.exit(f"horizon must be positive, not {piece!r}")
    if not horizons:
        sys.exit("give at least one horizon")
    return horizons


def parse_swings(text):
    swings = []
    for piece in text.split(","):
        item = piece.strip()
        if not item:
            continue
        try:
            swings.append(float(item))
        except ValueError:
            sys.exit(f"bad temperature swing {piece!r}")
    return [swing for swing in swings if swing != 0]


def column(rows):
    width = max(len(row) for row in rows)
    padded = [row.ljust(width) for row in rows]
    return padded


def print_table(headers, data):
    grid = [headers] + data
    cols = len(headers)
    shown = []
    for index in range(cols):
        shown.append(column([row[index] for row in grid]))
    for row_i in range(len(grid)):
        print("  ".join(shown[col][row_i] for col in range(cols)))


def report(live, source, tracks, groups, directive, temp_rows, clockerror, clockerror_from, args):
    root_distance = live.root_delay / 2.0 + live.root_disp
    measured_ppm = live.skew + abs(live.residual)
    chrony_ppm = measured_ppm + clockerror
    usable_rows = [row for group in groups for row in group]
    print("chrony holdover")
    print(
        "NTP servers and refclocks are assumed to stop now. "
        "The last frequency stays in place."
    )
    if directive is not None:
        print("The configured tempcomp line keeps running. It does not need those sources.")
    else:
        print("No tempcomp line is active.")
    print()
    print(f"source of the live numbers: {source}")
    if live.when is not None:
        print(f"at: {fmt_time(live.when)}")
    stratum = f" stratum {live.stratum}" if live.stratum is not None else ""
    print(f"reference: {live.ref}{stratum}")
    if not live.synced:
        print("chrony does not currently claim to be synchronised")
    print(f"system time: {fmt_signed(live.offset)} of the reference")
    print(
        f"root distance: {fmt_abs(root_distance)} "
        f"(delay/2 {fmt_abs(live.root_delay / 2.0)} + dispersion {fmt_abs(live.root_disp)})"
    )
    print(
        f"frequency: {live.frequency:+.3f} ppm, "
        f"residual {live.residual:+.3f} ppm, "
        f"skew {live.skew:.3f} ppm"
    )
    print(
        f"maxclockerror: {clockerror:.3f} ppm ({clockerror_from}). "
        "This allowance is added on top of the measured skew."
    )
    print(
        f"measured growth: {measured_ppm:.3f} ppm "
        f"({fmt_abs(ppm_to_seconds(measured_ppm, 86400))} per day)"
    )
    print(
        f"chrony growth: {chrony_ppm:.3f} ppm "
        f"({fmt_abs(ppm_to_seconds(chrony_ppm, 86400))} per day)"
    )
    if tracks:
        print(
            f"tracking log: {len(tracks)} rows, "
            f"{fmt_time(tracks[0].t)} .. {fmt_time(tracks[-1].t)}"
        )
    print(
        f"NTP samples used for the observed column: {len(usable_rows)} "
        f"in {len(groups)} stretches"
    )
    if usable_rows:
        freqs = [row.freq for row in usable_rows]
        print(
            f"frequency of those samples: {min(freqs):+.3f} .. {max(freqs):+.3f} ppm "
            f"(span {max(freqs) - min(freqs):.3f} ppm)"
        )
    print()
    print(
        "point is the current offset plus the residual frequency held for the whole horizon."
    )
    print(
        "measured bound is the root distance plus the measured growth. "
        "chrony bound also adds maxclockerror."
    )
    print(
        "observed is the median frequency change in the log at that horizon, "
        "counted as a rate error for the whole horizon. A smooth ramp would "
        "accumulate about half of that. The 90% column is the same for the "
        "90th percentile of those changes."
    )
    print()
    headers = [
        "horizon",
        "point",
        "measured bound",
        "chrony bound",
        "observed median",
        "observed 90%",
    ]
    data = []
    short = []
    for horizon in args.horizons:
        point = live.offset + ppm_to_seconds(live.residual, horizon)
        measured = root_distance + ppm_to_seconds(measured_ppm, horizon)
        chrony = root_distance + ppm_to_seconds(chrony_ppm, horizon)
        moves = frequency_moves(groups, horizon)
        if len(moves) >= args.min_pairs:
            mid = percentile(moves, 0.50)
            high = percentile(moves, 0.90)
            observed = fmt_abs(ppm_to_seconds(mid, horizon))
            observed_90 = fmt_abs(ppm_to_seconds(high, horizon))
            observed += f" ({mid:.3f} ppm, n={len(moves)})"
        else:
            observed = "log too short"
            observed_90 = f"n={len(moves)}"
            short.append(horizon)
        data.append(
            [
                fmt_span(horizon),
                fmt_signed(point),
                fmt_abs(measured),
                fmt_abs(chrony),
                observed,
                observed_90,
            ]
        )
    print_table(headers, data)
    if short:
        print()
        if args.include_rotated:
            print(
                "Horizons marked 'log too short' need "
                f"{args.min_pairs} pairs about that far apart, "
                "including rotated tracking logs."
            )
        else:
            print(
                "Horizons marked 'log too short' need "
                f"{args.min_pairs} pairs about that far apart in tracking.log."
            )

    print()
    print("temperature")
    if directive is None:
        print("No tempcomp line. The table above is the whole estimate.")
        return
    print(f"configured: {directive.raw}")
    raw, err = read_sensor(directive.sensor)
    if raw is None:
        print(f"sensor {directive.sensor} is not readable ({err})")
        comp_now = None
    else:
        try:
            comp_now = poly_at(raw, directive)
        except (TypeError, ValueError):
            comp_now = None
            print(f"sensor {directive.sensor} is {display_temp(raw):.1f} °C")
            print("the compensation at that temperature could not be calculated")
        else:
            print(
                f"sensor {directive.sensor} is {display_temp(raw):.1f} °C, "
                f"compensation {comp_now:+.3f} ppm"
            )
            per_c = slope_ppm_per_c(directive, raw)
            if per_c is not None:
                per_f = per_c * F_TO_C
                print(
                    f"slope at this temperature: {per_c:+.3f} ppm per °C "
                    f"({per_f:+.3f} ppm per °F, "
                    f"{fmt_abs(ppm_to_seconds(abs(per_f), 3600))} per hour per °F)"
                )
    if temp_rows:
        temps = [item[1] for item in temp_rows]
        comps = [item[2] for item in temp_rows]
        print(
            f"tempcomp.log: {len(temp_rows)} samples, "
            f"{fmt_time(temp_rows[0][0])} .. {fmt_time(temp_rows[-1][0])}, "
            f"{display_temp(min(temps)):.1f} .. {display_temp(max(temps)):.1f} °C, "
            f"compensation {min(comps):+.3f} .. {max(comps):+.3f} ppm"
        )
        span_c = abs(display_temp(max(temps)) - display_temp(min(temps)))
        if span_c < 1.0 or (temp_rows[-1][0] - temp_rows[0][0]).total_seconds() < 3600:
            print(
                "That temperature record is short, so it does not yet show "
                "how much of the frequency movement is left after tempcomp."
            )
    else:
        print("tempcomp.log has no samples. The slope above is the configured model.")
    print()
    print(
        "The measured skew was taken with tempcomp already applied, so the "
        "table above is the holdover with that line left on."
    )
    if comp_now is None or not args.swings:
        return
    print()
    print(
        "Tempcomp stays on. If the sensor moves and then stays there, the "
        "compensation changes by the amount below. The 1 day and 7 day columns "
        "are how far that change walks the clock. The model is cancelling the "
        "crystal's temperature drift, so this walk is not added to the measured bound."
    )
    headers = ["change", "comp change", "1 day", "7 days"]
    data = []
    # Each requested magnitude is shown in both directions. The model is
    # not symmetric around the current temperature.
    swings = sorted({-abs(swing) for swing in args.swings} | {abs(swing) for swing in args.swings})
    for swing_f in swings:
        delta_c = swing_f * F_TO_C
        # Swing is in °C once converted. Millidegree sensors store 1000 units per degree.
        step = delta_c * 1000.0 if abs(raw) >= 200 else delta_c
        try:
            delta_ppm = poly_at(raw + step, directive) - comp_now
        except (TypeError, ValueError):
            continue
        data.append(
            [
                f"{swing_f:+g} °F",
                f"{delta_ppm:+.3f} ppm",
                fmt_signed(ppm_to_seconds(delta_ppm, DAY)),
                fmt_signed(ppm_to_seconds(delta_ppm, WEEK)),
            ]
        )
    if data:
        print_table(headers, data)


def run(args):
    live, err = read_chronyc()
    source = "chronyc tracking"
    paths = log_files(args.log_dir, "tracking", args.include_rotated)
    rows = load_tracking(paths) if paths else []
    if live is None:
        if not rows:
            sys.exit(err or "chronyc tracking failed, and tracking.log is empty")
        live = live_from_log(rows[-1])
        source = f"last line of {paths[-1]} ({err})"
        print(f"note: {err}; using {source}", file=sys.stderr)
    windows = startup_windows(rows, args.startup)
    groups = segments(rows, windows, args.max_gap)
    clockerror, clockerror_from = load_maxclockerror(args.chrony_conf)
    directive = load_directive(args.chrony_conf)
    temp_paths = log_files(args.log_dir, "tempcomp", args.include_rotated)
    temp_rows = load_tempcomp(temp_paths) if temp_paths else []
    report(
        live,
        source,
        rows,
        groups,
        directive,
        temp_rows,
        clockerror,
        clockerror_from,
        args,
    )
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Estimate system-clock holdover error from chrony tracking.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  ./holdover-estimate.py\n"
            "  ./holdover-estimate.py --no-include-rotated\n"
            "  ./holdover-estimate.py --temp-swing 2,5,10\n"
            "\n"
            "The measured bound uses the live skew and residual frequency.\n"
            "The chrony bound also adds maxclockerror (1 ppm unless chrony.conf\n"
            "sets it). The observed column comes from tracking.log, including\n"
            "rotated logs unless --no-include-rotated is set. The temperature\n"
            "table keeps tempcomp on and shows 1 day and 7 day compensation\n"
            "walks for sensor moves in °F.\n"
        ),
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("/var/log/chrony"),
        help="directory with tracking.log and tempcomp.log (default /var/log/chrony)",
    )
    parser.add_argument(
        "--chrony-conf",
        type=Path,
        default=Path("/etc/chrony/chrony.conf"),
        help="config used for tempcomp and maxclockerror (default /etc/chrony/chrony.conf)",
    )
    parser.add_argument(
        "--include-rotated",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="read tracking.log.N and tempcomp.log.N (default on)",
    )
    parser.add_argument(
        "--startup",
        type=float,
        default=900.0,
        help="seconds of NTP samples to skip after an unsynchronised sample (default 900)",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=3600.0,
        help="split a stretch of NTP samples when a gap exceeds this many seconds (default 3600)",
    )
    parser.add_argument(
        "--horizons",
        default="1m,10m,1h,6h,1d,7d",
        help="comma-separated holdover durations (default 1m,10m,1h,6h,1d,7d)",
    )
    parser.add_argument(
        "--temp-swing",
        default="2,5,10",
        help="temperature moves in °F with tempcomp left on; both directions are printed (default 2,5,10)",
    )
    parser.add_argument(
        "--min-pairs",
        type=int,
        default=5,
        help="frequency-change pairs required before an observed value is shown (default 5)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    args.horizons = parse_horizons(args.horizons)
    args.swings = parse_swings(args.temp_swing)
    if args.startup < 0 or args.max_gap < 0:
        sys.exit("startup and max-gap must be >= 0")
    if args.min_pairs < 1:
        sys.exit("min-pairs must be at least 1")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
