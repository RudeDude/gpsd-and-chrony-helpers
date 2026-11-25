#!/usr/bin/env python3
"""Recommend chrony offset and delay for USB GPS refclocks.

Reads chrony's refclocks.log and tracking.log. A refclock sample counts
only when all of these are true:

- an NTP server is the selected source (not a refclock, and not 0.0.0.0)
- the tracking offset is under 1 ms, so the local clock already matches NTP
- the sample is past the warmup after a chronyd start (default 15 min)
- the sample is past the settle time after a refclock stopped being the
  selected source (default 90 s)

The raw column is the sample before the refclock offset option. Across a
steady configuration, the median of cooked minus raw is that configured
offset. The offset that centers the refclock on the NTP servers is the
negation of the median raw sample, rounded to 0.1 ms.

delay/2 is the +/- bar chrony uses when it picks a source. It is the chosen
percentile of the filtered (poll-combined) residuals after the new offset,
rounded up to the next millisecond. poll, filter, precision, and
maxdispersion are left as they are: the SHM dispersion column is the
driver's timestamp noise, and the USB jitter shows up in the residual.

The script prints a refclock line. The last lines of the report are those
lines alone, so they can be pasted into chrony.conf. It does not write
the file.
"""

import argparse
import ipaddress
import math
import re
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from decimal import ROUND_HALF_UP, Decimal
from pathlib import Path

DATE_RE = re.compile(r"\d{4}-\d{2}-\d{2}")

# cooked-raw wanders by a few tenths of a millisecond on SHM. A step larger
# than this is a real offset change inside one log, and the earlier regime
# is a different configuration.
REGIME_STEP = 0.002


@dataclass
class Sample:
    t: datetime
    refid: str
    raw: float | None
    cooked: float
    disp: float

    @property
    def filtered(self):
        return self.raw is None


@dataclass
class Track:
    t: datetime
    ref: str
    offset: float
    root_delay: float


def parse_stamp(date_s, time_s):
    """Parse a chrony log timestamp. The files are UTC. Naive conversion would
    shift the labels by the local offset."""
    text = f"{date_s} {time_s}"
    if "." in time_s:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S.%f")
    else:
        stamp = datetime.strptime(text, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(tzinfo=timezone.utc)


def parse_ip(text):
    try:
        return ipaddress.ip_address(text)
    except ValueError:
        return None


def is_ntp(ref):
    addr = parse_ip(ref)
    return addr is not None and not addr.is_unspecified


def is_unspecified(ref):
    addr = parse_ip(ref)
    return addr is not None and addr.is_unspecified


def round_half_up(value, places):
    quant = Decimal(1).scaleb(-places)
    rendered = Decimal(f"{value:.10f}")
    return float(rendered.quantize(quant, rounding=ROUND_HALF_UP))


def fmt_offset(value):
    return f"{round_half_up(value, 4):.4f}"


def fmt_delay(value):
    return f"{value:.3f}"


def fmt_ms(seconds):
    return f"{seconds * 1000:.2f} ms"


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


def percentile_rank(count, pct):
    """1-based rank of this percentile. Rank == count means the largest sample."""
    if pct >= 100:
        return count
    rank = math.ceil(pct / 100.0 * count - 1e-12)
    return min(max(rank, 1), count)


def percentile(values, pct):
    """Smallest sample at or above this percent of the list. pct 100 is the max."""
    if not values:
        raise ValueError("empty")
    ordered = sorted(values)
    return ordered[percentile_rank(len(ordered), pct) - 1]


def delay_for(level):
    """delay whose +/- bar is the next millisecond at or above level.

    Half of delay is the error chrony feeds the source-selection algorithm.
    The smallest bar is 1 ms (delay 0.002). The driver default, 1 ns, lets a
    quiet GPS outvote the NTP servers.
    """
    half_ms = math.ceil(level * 1000.0 - 1e-6)
    half_ms = max(1, half_ms)
    return half_ms * 2 / 1000.0, half_ms


def log_files(directory, stem, rotated):
    current = directory / f"{stem}.log"
    files = []
    if rotated:
        numbered = []
        for path in directory.glob(f"{stem}.log.*"):
            suffix = path.name.split(".log.", 1)[1]
            if suffix.isdigit():
                numbered.append((int(suffix), path))
        # Higher suffixes are older. Read them first.
        files.extend(path for _, path in sorted(numbered, reverse=True))
    if current.exists():
        files.append(current)
    return files


def read_text(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        sys.exit(f"cannot read {path}: {exc}")


def load_tracking(paths):
    tracks = []
    for path in paths:
        for line in read_text(path):
            parts = line.split()
            if len(parts) < 12 or not DATE_RE.fullmatch(parts[0]):
                continue
            try:
                tracks.append(
                    Track(
                        parse_stamp(parts[0], parts[1]),
                        parts[2],
                        float(parts[6]),
                        float(parts[11]),
                    )
                )
            except ValueError:
                continue
    tracks.sort(key=lambda tr: tr.t)
    return tracks


def load_refclocks(paths):
    samples = []
    for path in paths:
        for line in read_text(path):
            parts = line.split()
            if len(parts) < 9 or not DATE_RE.fullmatch(parts[0]):
                continue
            try:
                when = parse_stamp(parts[0], parts[1])
                cooked = float(parts[7])
                disp = float(parts[8])
            except ValueError:
                continue
            raw = None if parts[6] == "-" else _float_or_skip(parts[6])
            if parts[6] != "-" and raw is None:
                continue
            samples.append(Sample(when, parts[2], raw, cooked, disp))
    samples.sort(key=lambda sample: sample.t)
    return samples


def _float_or_skip(text):
    try:
        return float(text)
    except ValueError:
        return None


def merge_windows(windows):
    if not windows:
        return []
    ordered = sorted(windows)
    merged = [[ordered[0][0], ordered[0][1]]]
    for start, end in ordered[1:]:
        if start <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return [(start, end) for start, end in merged]


def window_overlap(start, end, windows):
    total = 0.0
    for win_start, win_end in windows:
        lo = max(start, win_start)
        hi = min(end, win_end)
        if hi > lo:
            total += (hi - lo).total_seconds()
    return total


def in_windows(stamp, windows):
    for start, end in windows:
        if stamp < start:
            return False
        if stamp < end:
            return True
    return False


class Timeline:
    def __init__(self, tracks, startup_s, settle_s, max_offset):
        self.tracks = tracks
        self.times = [tr.t for tr in tracks]
        self.max_offset = max_offset
        startup = []
        settle = []
        if startup_s > 0:
            for tr in tracks:
                if is_unspecified(tr.ref):
                    startup.append((tr.t, tr.t + timedelta(seconds=startup_s)))
        if settle_s > 0:
            for i, tr in enumerate(tracks[:-1]):
                # Settle starts when a refclock hands the clock back to NTP.
                if is_ntp(tr.ref) or is_unspecified(tr.ref):
                    continue
                nxt = tracks[i + 1]
                if is_ntp(nxt.ref):
                    settle.append((nxt.t, nxt.t + timedelta(seconds=settle_s)))
        self.startup = merge_windows(startup)
        self.settle = merge_windows(settle)
        self.blocked = merge_windows(self.startup + self.settle)

    def covering(self, stamp):
        if not self.times:
            return None
        # Last tracking update at or before this sample.
        idx = _bisect_right(self.times, stamp) - 1
        if idx < 0:
            return None
        return self.tracks[idx]

    def classify(self, stamp):
        tr = self.covering(stamp)
        if tr is None:
            return "before-tracking", None
        if in_windows(stamp, self.startup):
            return "warmup", tr
        if not is_ntp(tr.ref):
            return "unsynced", tr
        if in_windows(stamp, self.settle):
            return "settle", tr
        if abs(tr.offset) >= self.max_offset:
            return "tracking-offset", tr
        return "ok", tr


def _bisect_right(times, stamp):
    lo, hi = 0, len(times)
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] <= stamp:
            lo = mid + 1
        else:
            hi = mid
    return lo


def selection_durations(tracks, log_end):
    totals = {}
    if not tracks:
        return totals
    end_bound = max(log_end, tracks[-1].t)
    for i, tr in enumerate(tracks):
        nxt = tracks[i + 1].t if i + 1 < len(tracks) else end_bound
        dt = (nxt - tr.t).total_seconds()
        if dt <= 0:
            continue
        label = "NTP" if is_ntp(tr.ref) else tr.ref
        totals[label] = totals.get(label, 0.0) + dt
    return totals


def usable_ntp_seconds(tracks, timeline, log_end):
    if not tracks:
        return 0.0, []
    end_bound = max(log_end, tracks[-1].t)
    usable = 0.0
    root_delays = []
    for i, tr in enumerate(tracks):
        if not is_ntp(tr.ref) or abs(tr.offset) >= timeline.max_offset:
            continue
        nxt = tracks[i + 1].t if i + 1 < len(tracks) else end_bound
        if nxt <= tr.t:
            continue
        span = (nxt - tr.t).total_seconds()
        blocked = window_overlap(tr.t, nxt, timeline.blocked)
        kept = span - blocked
        if kept > 0:
            usable += kept
            root_delays.append(tr.root_delay)
    return usable, root_delays


def current_regime(samples):
    """Drop raw samples left over from an earlier offset setting.

    Returns the samples that match the latest configured offset, that offset
    (median of cooked-raw), and how many samples were left out.
    """
    if not samples:
        return [], None, 0
    deltas = [sample.cooked - sample.raw for sample in samples]
    if max(deltas) - min(deltas) <= REGIME_STEP:
        return samples, statistics.median(deltas), 0
    tail_n = max(20, len(samples) // 5)
    center = statistics.median(deltas[-tail_n:])
    kept = [
        sample
        for sample, delta in zip(samples, deltas)
        if abs(delta - center) <= REGIME_STEP
    ]
    if not kept:
        kept = samples[-tail_n:]
    offset = statistics.median(sample.cooked - sample.raw for sample in kept)
    return kept, offset, len(samples) - len(kept)


def _refid_key(refid):
    if refid.upper().startswith("GPS") and refid[3:].isdigit():
        return (0, int(refid[3:]))
    return (1, refid)


def is_wanted(refid, explicit, include_pps):
    if explicit is not None:
        return refid in explicit
    if refid.upper().startswith("PPS"):
        return include_pps
    return refid.upper().startswith("GPS")


def empty_drops():
    return {
        "before-tracking": 0,
        "warmup": 0,
        "unsynced": 0,
        "settle": 0,
        "tracking-offset": 0,
    }


def analyze_refid(refid, samples, timeline, args):
    drops_raw = empty_drops()
    drops_filt = empty_drops()
    raw_ok = []
    filt_ok = []
    for sample in samples:
        reason, _track = timeline.classify(sample.t)
        if sample.filtered:
            if reason == "ok":
                filt_ok.append(sample)
            else:
                drops_filt[reason] += 1
        else:
            if reason == "ok":
                raw_ok.append(sample)
            else:
                drops_raw[reason] += 1

    kept, log_offset, regime_dropped = current_regime(raw_ok)
    if regime_dropped and kept:
        start, end = kept[0].t, kept[-1].t
        filt_ok = [sample for sample in filt_ok if start <= sample.t <= end]

    result = {
        "refid": refid,
        "raw_total": sum(1 for sample in samples if not sample.filtered),
        "filt_total": sum(1 for sample in samples if sample.filtered),
        "drops_raw": drops_raw,
        "drops_filt": drops_filt,
        "regime_dropped": regime_dropped,
        "log_offset": log_offset,
        "raw_n": len(kept),
        "filt_n": len(filt_ok),
        "span": 0.0,
        "status": "short",
        "offset": None,
        "delay": None,
        "half_ms": None,
        "raw_median": None,
        "raw_stdev": None,
        "raw_min": None,
        "raw_max": None,
        "levels": [],
    }
    if not kept:
        return result

    raws = [sample.raw for sample in kept]
    result["span"] = (kept[-1].t - kept[0].t).total_seconds()
    result["t0"] = kept[0].t
    result["t1"] = kept[-1].t
    result["raw_median"] = statistics.median(raws)
    result["raw_min"] = min(raws)
    result["raw_max"] = max(raws)
    if len(raws) >= 2:
        result["raw_stdev"] = statistics.stdev(raws)
    result["offset"] = round_half_up(-result["raw_median"], 4)

    if not filt_ok or log_offset is None:
        result["status"] = "no-filtered"
        return result

    shift = result["offset"] - log_offset
    abs_resid = [abs(sample.cooked + shift) for sample in filt_ok]
    percents = [args.cover]
    for extra in (95, 99, 100):
        if extra not in percents:
            percents.append(extra)
    levels = []
    for pct in percents:
        level = percentile(abs_resid, pct)
        delay, half_ms = delay_for(level)
        levels.append({"pct": pct, "level": level, "delay": delay, "half_ms": half_ms})
    result["cover"] = args.cover
    result["levels"] = sorted(levels, key=lambda item: item["pct"])
    chosen = next(item for item in result["levels"] if item["pct"] == args.cover)
    result["delay"] = chosen["delay"]
    result["half_ms"] = chosen["half_ms"]
    long_enough = (
        result["raw_n"] >= args.min_samples
        and result["filt_n"] >= args.min_filtered
        and result["span"] >= args.min_span
    )
    result["status"] = "ok" if long_enough else "preliminary"
    return result


def load_refclock_lines(path):
    """Active refclock lines keyed by refid. Commented history is ignored."""
    if path is None:
        return {}
    if not path.exists():
        return {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        print(f"chrony.conf is not readable ({exc})", file=sys.stderr)
        return {}
    found = {}
    for line in lines:
        body = line.strip()
        if not body or body.startswith("#"):
            continue
        parts = body.split()
        if not parts or parts[0] != "refclock":
            continue
        refid = None
        for i, tok in enumerate(parts):
            if tok == "refid" and i + 1 < len(parts):
                refid = parts[i + 1]
                break
        if refid is None:
            continue
        found.setdefault(refid, []).append(line.rstrip("\n"))
    return found


def split_comment(line):
    idx = line.find("#")
    if idx == -1:
        return line, ""
    return line[:idx], line[idx:]


def replace_option(line, key, rendered):
    code, comment = split_comment(line)
    pattern = re.compile(rf"(\b{key}\s+)(\S+)")
    if pattern.search(code):
        code = pattern.sub(lambda match: match.group(1) + rendered, code, count=1)
    else:
        code = code.rstrip() + f" {key} {rendered}"
    return code + comment


def suggest_line(line, offset, delay):
    updated = replace_option(line, "offset", fmt_offset(offset))
    return replace_option(updated, "delay", fmt_delay(delay))


def option_value(line, key):
    code, _comment = split_comment(line)
    match = re.search(rf"\b{key}\s+(\S+)", code)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def cover_text(pct):
    if pct >= 100:
        return "max"
    return f"p{pct:g}"


def print_report(context, results, args, conf_lines):
    print("chrony GPS offset")
    print(f"log dir: {args.log_dir}")
    if context["files"]:
        print("files: " + ", ".join(context["files"]))
    if context["t0"] is not None:
        print(f"span: {fmt_time(context['t0'])} .. {fmt_time(context['t1'])}")
    print(
        "filters: NTP selected, "
        f"|tracking offset| < {args.max_offset:g} s, "
        f"warmup {args.startup:g} s after chronyd start, "
        f"settle {args.settle:g} s after a refclock selection, "
        f"cover {cover_text(args.cover)}"
    )
    if context["durations"]:
        pieces = [
            f"{label} {fmt_dur(seconds)}"
            for label, seconds in sorted(
                context["durations"].items(), key=lambda item: -item[1]
            )
        ]
        print("selected: " + ", ".join(pieces))
    print(f"usable NTP inside the filters: {fmt_dur(context['usable'])}")
    for start, end in context["warmups"]:
        print(f"warmup: {fmt_time(start)} .. {fmt_time(end)}")
    if context["ignored"]:
        print("left out refids: " + ", ".join(context["ignored"]))
    print()

    for result in results:
        _print_refid(result, context, conf_lines)
    for line in conf_text(results, conf_lines):
        print(line)


def _print_refid(result, context, conf_lines):
    refid = result["refid"]
    print(refid)
    print(
        f"  raw:      {result['raw_total']} in the log, {result['raw_n']} used"
    )
    print("            " + _drop_text(result["drops_raw"]))
    print(
        f"  filtered: {result['filt_total']} in the log, {result['filt_n']} used"
    )
    print("            " + _drop_text(result["drops_filt"]))
    if result["regime_dropped"]:
        print(
            f"  left out {result['regime_dropped']} raw samples from an earlier "
            f"offset (cooked-raw stepped by more than {REGIME_STEP * 1000:.0f} ms)"
        )
    if result["status"] == "short":
        print("  status: short")
        print("  no recommendation yet. The clean NTP window has no raw samples.")
        _print_configured(refid, conf_lines, None, None, False)
        print()
        return

    print(
        f"  used:     {fmt_time(result['t0'])} .. {fmt_time(result['t1'])} "
        f"({fmt_dur(result['span'])})"
    )
    spread = f"stdev {fmt_ms(result['raw_stdev'])}" if result["raw_stdev"] is not None else "stdev n/a"
    print(
        f"  raw median {fmt_ms(result['raw_median'])}, {spread}, "
        f"min {fmt_ms(result['raw_min'])}, max {fmt_ms(result['raw_max'])}"
    )
    if result["log_offset"] is not None:
        print(f"  log offset (median cooked-raw): {fmt_offset(result['log_offset'])} s")
    print(f"  recommended offset: {fmt_offset(result['offset'])} s")

    if result["status"] == "no-filtered":
        print("  status: no-filtered")
        print("  delay needs poll-combined samples in the clean window. None yet.")
        _print_configured(refid, conf_lines, result["offset"], None, False)
        print()
        return

    same_delay = len({item["delay"] for item in result["levels"]}) == 1
    if same_delay:
        item = next(level for level in result["levels"] if level["pct"] == result_cover(result))
        bits = ", ".join(
            f"{cover_text(level['pct'])} {fmt_ms(level['level'])}" for level in result["levels"]
        )
        print(
            f"  filtered |residual|: {bits} -> delay {fmt_delay(item['delay'])} "
            f"(+/- {item['half_ms']} ms)"
        )
    else:
        print("  filtered |residual|:")
        for level in result["levels"]:
            mark = "  [recommended]" if level["pct"] == result_cover(result) else ""
            print(
                f"    {cover_text(level['pct']):<4} {fmt_ms(level['level']):>10} "
                f"-> delay {fmt_delay(level['delay'])} (+/- {level['half_ms']} ms){mark}"
            )
    print(f"  status: {result['status']}")
    for line in _notes(result, context):
        print(f"  note: {line}")
    _print_configured(
        refid, conf_lines, result["offset"], result["delay"], result["status"] == "ok"
    )
    print()


def conf_text(results, conf_lines):
    """Refclock lines with the recommended offset and delay filled in."""
    lines = []
    for result in results:
        offset = result["offset"]
        delay = result["delay"]
        if offset is None or delay is None:
            continue
        for line in conf_lines.get(result["refid"], []):
            lines.append(suggest_line(line, offset, delay).strip())
    return lines


def result_cover(result):
    return result["cover"]


def _drop_text(drops):
    return (
        "left out: "
        f"warmup {drops['warmup']}, unsynced {drops['unsynced']}, "
        f"settle {drops['settle']}, tracking-offset {drops['tracking-offset']}, "
        f"before-tracking {drops['before-tracking']}"
    )


def _notes(result, context):
    notes = []
    selected = context["durations"].get(result["refid"], 0.0)
    if selected >= 60:
        notes.append(
            f"{result['refid']} was the selected source for {fmt_dur(selected)}. "
            "Those samples are left out of this estimate."
        )
    root = context["root_delay"]
    if root is not None and result["half_ms"] is not None:
        # A refclock's selection distance is about delay/2. An NTP source's
        # distance includes about half of its root delay, plus its own delay.
        half = result["half_ms"] / 1000.0
        if half < root / 2.0:
            wider = ""
            by_pct = {level["pct"]: level for level in result["levels"]}
            if 100 in by_pct and by_pct[100]["delay"] > result["delay"] + 1e-9:
                wider = (
                    " Covering the largest residual needs delay "
                    f"{fmt_delay(by_pct[100]['delay'])}."
                )
            notes.append(
                f"The +/- bar is {result['half_ms']} ms, under half the NTP "
                f"root delay in this window ({fmt_ms(root)}). A quiet stretch "
                "can make this refclock the selected source." + wider
            )
    if result["levels"]:
        by_pct = {level["pct"]: level for level in result["levels"]}
        if 99 in by_pct and 100 in by_pct and result_cover(result) < 100:
            p99 = by_pct[99]
            widest = by_pct[100]
            if widest["level"] > p99["level"] + 0.002:
                notes.append(
                    f"The largest filtered residual is {fmt_ms(widest['level'])} "
                    f"(p99 is {fmt_ms(p99['level'])}). Covering the largest "
                    f"needs delay {fmt_delay(widest['delay'])}."
                )
    if (
        result["filt_n"]
        and result_cover(result) < 100
        and percentile_rank(result["filt_n"], result_cover(result)) >= result["filt_n"]
    ):
        notes.append(
            f"With {result['filt_n']} filtered samples, "
            f"{cover_text(result_cover(result))} is the largest residual "
            "in the window."
        )
    if result["status"] == "preliminary":
        unmet = []
        if result["span"] < context["min_span"]:
            unmet.append(
                f"clean span is {fmt_dur(result['span'])} "
                f"and the bar for a finished estimate is {fmt_dur(context['min_span'])}"
            )
        if result["raw_n"] < context["min_samples"]:
            unmet.append(
                f"raw samples are {result['raw_n']} "
                f"and the bar is {context['min_samples']}"
            )
        if result["filt_n"] < context["min_filtered"]:
            unmet.append(
                f"filtered samples are {result['filt_n']} "
                f"and the bar is {context['min_filtered']}"
            )
        detail = "; ".join(unmet) if unmet else "the window is still short"
        notes.append(f"Preview only. {detail[0].upper() + detail[1:]}.")
    return notes


def _print_configured(refid, conf_lines, offset, delay, ready):
    lines = conf_lines.get(refid, [])
    if not lines:
        if offset is not None and delay is not None:
            tag = "suggested" if ready else "preview"
            print(
                f"  {tag}: refclock <driver> <param> refid {refid} "
                f"offset {fmt_offset(offset)} delay {fmt_delay(delay)}"
            )
        return
    if len(lines) > 1:
        print(f"  {len(lines)} active refclock lines use refid {refid}")
    for line in lines:
        print(f"  configured: {line.strip()}")
        if offset is None or delay is None:
            continue
        suggested = suggest_line(line, offset, delay)
        old_offset = option_value(line, "offset")
        old_delay = option_value(line, "delay")
        same = suggested.strip() == line.strip()
        tag = "suggested" if ready else "preview"
        if same:
            print(f"  {tag}: matches the configured line")
        else:
            print(f"  {tag}: {suggested.strip()}")
            if old_offset is not None and abs(old_offset - offset) >= 0.0005:
                print(
                    f"  offset change {fmt_offset(old_offset)} -> {fmt_offset(offset)}"
                )
            if old_delay is not None and abs(old_delay - delay) >= 0.0005:
                print(
                    f"  delay change {fmt_delay(old_delay)} -> {fmt_delay(delay)}"
                )


def build_context(tracks, samples, timeline, args, ignored):
    stamps = [tr.t for tr in tracks] + [sample.t for sample in samples]
    log_end = max(stamps) if stamps else None
    log_start = min(stamps) if stamps else None
    usable, root_delays = usable_ntp_seconds(
        tracks, timeline, log_end or datetime.now(timezone.utc)
    )
    files = []
    for stem in ("refclocks", "tracking"):
        for path in log_files(args.log_dir, stem, args.include_rotated):
            files.append(path.name)
    return {
        "t0": log_start,
        "t1": log_end,
        "durations": selection_durations(tracks, log_end) if log_end else {},
        "usable": usable,
        "root_delay": statistics.median(root_delays) if root_delays else None,
        "warmups": timeline.startup,
        "ignored": ignored,
        "files": files,
        "min_span": args.min_span,
        "min_samples": args.min_samples,
        "min_filtered": args.min_filtered,
    }


def wanted_refids(samples, args):
    present = []
    seen = set()
    for sample in samples:
        if sample.refid not in seen:
            seen.add(sample.refid)
            present.append(sample.refid)
    explicit = set(args.refid) if args.refid else None
    chosen = [refid for refid in present if is_wanted(refid, explicit, args.include_pps)]
    chosen.sort(key=_refid_key)
    ignored = [refid for refid in present if refid not in chosen]
    missing = []
    if explicit is not None:
        missing = sorted(explicit - set(present))
    return chosen, ignored, missing


def run(args):
    ref_paths = log_files(args.log_dir, "refclocks", args.include_rotated)
    tr_paths = log_files(args.log_dir, "tracking", args.include_rotated)
    if not ref_paths:
        sys.exit(f"no refclocks.log in {args.log_dir}")
    if not tr_paths:
        sys.exit(f"no tracking.log in {args.log_dir}")
    tracks = load_tracking(tr_paths)
    samples = load_refclocks(ref_paths)
    if not tracks:
        sys.exit("tracking log has no samples")
    if not samples:
        sys.exit("refclock log has no samples")
    chosen, ignored, missing = wanted_refids(samples, args)
    if missing:
        sys.exit("refid not in the refclock log: " + ", ".join(missing))
    if not chosen:
        extra = ""
        if any(refid.upper().startswith("PPS") for refid in ignored):
            extra = " Pass --include-pps to include PPS refids."
        sys.exit("no GPS refids in the refclock log." + extra)

    timeline = Timeline(tracks, args.startup, args.settle, args.max_offset)
    grouped = {refid: [] for refid in chosen}
    for sample in samples:
        if sample.refid in grouped:
            grouped[sample.refid].append(sample)
    results = [
        analyze_refid(refid, grouped[refid], timeline, args) for refid in chosen
    ]
    conf_path = args.chrony_conf
    conf_lines = load_refclock_lines(conf_path) if conf_path else {}
    context = build_context(tracks, samples, timeline, args, ignored)
    print_report(context, results, args, conf_lines)
    if args.strict and any(result["status"] != "ok" for result in results):
        return 2
    return 0


def parse_cover(text):
    if text == "max":
        return 100.0
    try:
        pct = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("cover is a percentile or 'max'") from exc
    if not 0 < pct <= 100:
        raise argparse.ArgumentTypeError("cover must be between 0 and 100, or 'max'")
    return pct


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Recommend chrony offset and delay for USB GPS refclocks.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  ./chrony-gps.py\n"
            "  ./chrony-gps.py --startup 0 --min-span 0\n"
            "  ./chrony-gps.py --cover max --refid GPS2\n"
            "\n"
            "A sample counts while an NTP server is selected. Time when a GPS\n"
            "refclock itself is the selected source is left out.\n"
        ),
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("/var/log/chrony"),
        help="directory with refclocks.log and tracking.log (default /var/log/chrony)",
    )
    parser.add_argument(
        "--chrony-conf",
        type=Path,
        default=Path("/etc/chrony/chrony.conf"),
        help="config used only to print a replacement refclock line (default /etc/chrony/chrony.conf)",
    )
    parser.add_argument(
        "--startup",
        type=float,
        default=900.0,
        help="seconds to skip after each chronyd start, seen as tracking ref 0.0.0.0 (default 900)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=90.0,
        help="seconds to skip after a refclock stops being the selected source (default 90)",
    )
    parser.add_argument(
        "--max-offset",
        type=float,
        default=0.001,
        help="skip samples while |tracking offset| is at least this many seconds (default 0.001)",
    )
    parser.add_argument(
        "--cover",
        type=parse_cover,
        default=99.0,
        help="percentile of |filtered residual| that delay/2 must cover, or 'max' (default 99)",
    )
    parser.add_argument(
        "--min-span",
        type=float,
        default=7200.0,
        help="clean-window seconds required before status is ok (default 7200)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=100,
        help="clean raw samples required before status is ok (default 100)",
    )
    parser.add_argument(
        "--min-filtered",
        type=int,
        default=20,
        help="clean filtered samples required before status is ok (default 20)",
    )
    parser.add_argument(
        "--refid",
        action="append",
        help="limit the report to this refid (repeatable). Default is every GPS* refid",
    )
    parser.add_argument(
        "--include-pps",
        action="store_true",
        help="also report PPS* refids",
    )
    parser.add_argument(
        "--include-rotated",
        action="store_true",
        help="also read refclocks.log.N and tracking.log.N",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="exit 2 unless every reported refid has status ok",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.startup < 0 or args.settle < 0 or args.max_offset < 0:
        sys.exit("startup, settle, and max-offset must be >= 0")
    if args.min_span < 0 or args.min_samples < 1 or args.min_filtered < 1:
        sys.exit("min-span must be >= 0 and the sample minimums must be >= 1")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
