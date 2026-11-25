#!/usr/bin/env python3
"""Save and restore u-blox aiding data through ubxtool.

u-blox 7 stores UBX-AID health, almanac, ephemeris, and AssistNow
Autonomous records. u-blox 8 and 9 store the navigation database as
UBX-MGA-DBD, which the receiver itself ages out. Position and time are
not taken from this file: load runs aid-ini.py so the clock is current.

One invocation talks to one gpsd device. The default cache file for
ttyACM0 is aiding-cache.json. Any other device gets its own file.

gpsd must already be talking to the receiver.
"""

import argparse
import json
import struct
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import gpsdev

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
WEEK_SECONDS = 604800

# How long each saved record is worth sending back.
EPH_HOURS = 3
AOP_DAYS = 3
HUI_DAYS = 180
ALM_WEEKS = 26

KIND = {
    "UBX-AID-HUI": (0x0B, 0x02),
    "UBX-AID-ALM": (0x0B, 0x30),
    "UBX-AID-EPH": (0x0B, 0x31),
    "UBX-AID-AOP": (0x0B, 0x33),
    "UBX-MGA-DBD": (0x13, 0x80),
}


def default_path(device):
    """ttyACM0 keeps the original cache name. Other devices get their own file."""
    directory = Path(__file__).resolve().parent
    if device == "/dev/ttyACM0":
        return directory / "aiding-cache.json"
    return directory / f"aiding-cache-{Path(device).name}.json"


def parse_aid_dump(text):
    """Pull UBX-AID payloads out of ubxtool -v 3 output."""
    found = []
    pending = None
    for line in text.splitlines():
        if line.startswith("payload:"):
            pending = line.split(":", 1)[1].strip()
            continue
        if not line.startswith("UBX-"):
            continue
        payload = pending
        pending = None
        if not payload or not (
            line.startswith("UBX-AID-") or line.startswith("UBX-MGA-")
        ):
            continue
        name = line.split()[0].rstrip(":")
        if name not in KIND:
            continue
        raw = bytes(int(part, 16) for part in payload.split(",") if part)
        if useful(name, raw):
            found.append({"name": name, "payload": payload})
    return found


def useful(name, raw):
    """Drop empty poll replies. An all-zero week or HOW means no orbit data."""
    if name == "UBX-AID-HUI":
        return len(raw) >= 72
    if name == "UBX-AID-ALM" and len(raw) >= 8:
        _svid, week = struct.unpack_from("<LL", raw)
        return week != 0 and len(raw) >= 40
    if name == "UBX-AID-EPH" and len(raw) >= 8:
        _svid, how = struct.unpack_from("<LL", raw)
        return how != 0 and len(raw) >= 104
    if name == "UBX-AID-AOP":
        return len(raw) > 8
    if name == "UBX-MGA-DBD":
        # Three reserved words, then one database record. The poll echo is empty.
        return len(raw) > 12
    return False


def alm_week(payload):
    raw = bytes(int(part, 16) for part in payload.split(","))
    return struct.unpack_from("<LL", raw)[1]


def gps_week(when):
    return int((when - GPS_EPOCH).total_seconds() // WEEK_SECONDS)


def current_fix(device):
    """Latest 2D or 3D fix from one gpsd device, or None."""
    proc = subprocess.run(
        ["timeout", "6", "gpspipe", "-w"],
        check=False,
        capture_output=True,
        text=True,
    )
    fix = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        if msg.get("class") != "TPV" or msg.get("mode", 0) < 2 or "lat" not in msg:
            continue
        if msg.get("device") not in (None, device):
            continue
        fix = msg
    return fix


def save(path, device, protver, protocol):
    if protocol == "mga" and not gpsdev.set_mga_ack(device, protver, False):
        sys.exit(f"{device} did not disable aiding acknowledgements for the poll")
    if protocol == "aid":
        polls = ["-p", "AID-HUI", "-p", "AID-ALM", "-p", "AID-EPH", "-p", "AID-AOP"]
        wait = "12"
    else:
        polls = ["-p", "MGA-DBD"]
        wait = "8"
    proc = subprocess.run(
        gpsdev.ubxtool_cmd(device, protver, "-v", "3", "-w", wait, *polls),
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        sys.exit(f"ubxtool exited {proc.returncode}")
    messages = parse_aid_dump(proc.stdout)
    if protocol == "mga":
        messages = [msg for msg in messages if msg["name"] == "UBX-MGA-DBD"]
    if not messages:
        sys.exit("no aiding messages in the ubxtool reply")

    now = datetime.now(timezone.utc)
    fix = current_fix(device)
    position = None
    if fix:
        position = {
            "lat": fix["lat"],
            "lon": fix["lon"],
            "alt": fix.get("alt", 180.0),
            "mode": fix.get("mode"),
            "epx": fix.get("epx"),
            "epy": fix.get("epy"),
        }
    doc = {
        "captured_at": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "device": device,
        "protver": protver,
        "protocol": protocol,
        "position": position,
        "messages": messages,
    }
    path.write_text(json.dumps(doc) + "\n")
    counts = {}
    for msg in messages:
        counts[msg["name"]] = counts.get(msg["name"], 0) + 1
    where = "no fix, position not stored"
    if position:
        where = (
            f"lat {position['lat']:.7f} lon {position['lon']:.7f} "
            f"alt {position['alt']:.1f} m mode {position['mode']}"
        )
    print(f"wrote {path}")
    print(f"captured {doc['captured_at']}  {where}")
    for name in KIND:
        if name in counts:
            print(f"  {name} {counts[name]}")


def send_payload(device, protver, protocol, cls, mid, payload):
    cmd = gpsdev.ubxtool_cmd(
        device,
        protver,
        "-v",
        "1",
        "-w",
        "0.4",
        "-c",
        f"{cls:02x},{mid:02x},{payload}",
    )
    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    text = proc.stdout + proc.stderr
    if proc.returncode != 0:
        return f"exit {proc.returncode}"
    if "UBX-ACK-NAK" in text:
        return "NAK"
    if protocol == "mga":
        codes = []
        marker = "infoCode "
        for line in text.splitlines():
            start = line.find(marker)
            if start < 0:
                continue
            number = line[start + len(marker):].split()[0].rstrip(",")
            if number.isdigit():
                codes.append(int(number))
        if not codes or any(code != 0 for code in codes):
            return "NAK"
    return "ok"


def load(path, device, protver, protocol):
    doc = json.loads(path.read_text())
    cached_protocol = doc.get("protocol", "aid")
    cached_device = doc.get("device")
    if cached_protocol != protocol:
        sys.exit(
            f"{path} is {cached_protocol} data and {device} speaks {protocol}"
        )
    if cached_device and cached_device != device:
        sys.exit(f"{path} was saved from {cached_device}, not {device}")

    if protocol == "mga" and not gpsdev.enable_mga_ack(device, protver):
        sys.exit(f"{device} did not enable aiding acknowledgements")

    captured = datetime.strptime(doc["captured_at"], "%Y-%m-%dT%H:%M:%SZ")
    captured = captured.replace(tzinfo=timezone.utc)
    age = datetime.now(timezone.utc) - captured
    now_week = gps_week(datetime.now(timezone.utc))
    position = doc.get("position")

    if position:
        ini = Path(__file__).resolve().parent / "aid-ini.py"
        # Fixed accuracy for the saved position. The receiver's own epx is
        # often much wider than the scatter of a stationary antenna.
        cmd = [
            sys.executable,
            str(ini),
            "--device",
            device,
            "--protver",
            gpsdev.protver_arg(protver),
            str(position["lat"]),
            str(position["lon"]),
            "--alt",
            str(position.get("alt", 180.0)),
            "--pos-acc",
            "30",
        ]
        print(
            f"fresh time at saved position ±30 m on {device}",
            file=sys.stderr,
        )
        proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
        sys.stderr.write(proc.stderr)
        if proc.returncode != 0 or "UBX-ACK-NAK" in proc.stdout:
            sys.exit(f"aid-ini.py exited {proc.returncode}")
    else:
        print("cache has no position; run aid-ini.py with a lat lon", file=sys.stderr)

    sent = {name: 0 for name in KIND}
    skipped = []
    naks = []
    outgoing = []
    for msg in doc["messages"]:
        name = msg["name"]
        if name not in KIND:
            skipped.append(name)
            continue
        if name == "UBX-AID-HUI" and age > timedelta(days=HUI_DAYS):
            skipped.append(f"HUI older than {HUI_DAYS} days")
            continue
        if name == "UBX-MGA-DBD" and age > timedelta(days=HUI_DAYS):
            skipped.append(f"navigation database older than {HUI_DAYS} days")
            continue
        if name == "UBX-AID-ALM":
            week = alm_week(msg["payload"])
            if abs(now_week - week) > ALM_WEEKS:
                skipped.append(f"almanac week {week}")
                continue
        if name == "UBX-AID-EPH" and age > timedelta(hours=EPH_HOURS):
            skipped.append("ephemeris")
            continue
        if name == "UBX-AID-AOP" and age > timedelta(days=AOP_DAYS):
            skipped.append("AOP")
            continue
        outgoing.append(msg)

    total = len(outgoing)
    for index, msg in enumerate(outgoing, start=1):
        name = msg["name"]
        cls, mid = KIND[name]
        if total > 20 and (index == 1 or index % 20 == 0 or index == total):
            print(f"  sending {index}/{total}", file=sys.stderr)
        result = send_payload(device, protver, protocol, cls, mid, msg["payload"])
        if result == "ok":
            sent[name] += 1
        else:
            naks.append(f"{name} {result}")

    print(f"cache age {age.total_seconds() / 3600:.1f} h")
    for name, count in sent.items():
        if count:
            print(f"  sent {name} {count}")
    if any(name == "UBX-AID-EPH" for name in (m["name"] for m in doc["messages"])):
        if sent["UBX-AID-EPH"] == 0:
            print(f"  ephemeris kept in the file, not sent after {EPH_HOURS} h")
    if any(name == "UBX-AID-AOP" for name in (m["name"] for m in doc["messages"])):
        if sent["UBX-AID-AOP"] == 0:
            print(f"  AOP kept in the file, not sent after {AOP_DAYS} days")
    if sent["UBX-MGA-DBD"] and age > timedelta(hours=EPH_HOURS):
        print("  database sent; the receiver drops ephemeris older than a few hours")
    for item in sorted(set(skipped)):
        if (
            item.startswith("almanac")
            or item.startswith("HUI")
            or item.startswith("navigation")
        ):
            print(f"  skipped {item}")
    if naks:
        print("rejected: " + ", ".join(naks))
        sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("action", choices=("save", "load"))
    ap.add_argument(
        "--device",
        default="/dev/ttyACM0",
        help="gpsd device (ttyACM0, ttyACM1, or a /dev path; default ttyACM0)",
    )
    ap.add_argument(
        "--protver",
        default=None,
        help="ubxtool protocol version (default: the version gpsd reports)",
    )
    ap.add_argument(
        "-f",
        "--file",
        type=Path,
        default=None,
        help="cache file (default aiding-cache.json on ttyACM0, else aiding-cache-<device>.json)",
    )
    args = ap.parse_args()
    device = gpsdev.canonical_device(args.device)
    info = gpsdev.describe(device)
    protver = float(args.protver) if args.protver is not None else info["protver"]
    protocol = gpsdev.protocol_for(protver)
    path = args.file if args.file is not None else default_path(device)
    print(
        f"{protocol} {device} protver {gpsdev.protver_arg(protver)}",
        file=sys.stderr,
    )
    if args.action == "save":
        save(path, device, protver, protocol)
    else:
        if not path.is_file():
            sys.exit(f"no cache at {path}")
        load(path, device, protver, protocol)


if __name__ == "__main__":
    main()
