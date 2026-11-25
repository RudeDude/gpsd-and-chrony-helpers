#!/usr/bin/env python3
"""Send a position and time hint through ubxtool.

u-blox 7 gets UBX-AID-INI. u-blox 8 and 9 get UBX-MGA-INI-POS_LLH and
UBX-MGA-INI-TIME_UTC. The time fields are taken from the system clock at
the moment the script runs. gpsd must already be talking to the receiver.
"""

import argparse
import struct
import subprocess
import sys
from datetime import datetime, timezone

import gpsdev

# UBX-AID-INI flags: position valid, time valid, lat/lon/alt.
# Time is GPS week and time-of-week. This u-blox 7 firmware ignores the UTC date form.
FLAG_POS = 0x01
FLAG_TIME = 0x02
FLAG_LLA = 0x20

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
WEEK_SECONDS = 604800

# UBX-MGA-INI-TIME_UTC: apply the timestamp when the message arrives.
# There is no time-pulse pin on these USB sticks.
MGA_TIME_ON_RECEIPT = 0x00
# Host clock is NTP-disciplined, so the receiver may use it for spoof checks.
MGA_TIME_TRUSTED = 0x01


def gps_week_tow_ms(when, leap_seconds):
    """GPS week and time-of-week in milliseconds for a UTC timestamp."""
    gps_seconds = (when - GPS_EPOCH).total_seconds() + leap_seconds
    if gps_seconds < 0:
        raise ValueError("time is before the GPS epoch")
    week = int(gps_seconds // WEEK_SECONDS)
    tow_ms = int(round((gps_seconds - week * WEEK_SECONDS) * 1000))
    if tow_ms >= WEEK_SECONDS * 1000:
        week += 1
        tow_ms -= WEEK_SECONDS * 1000
    return week, tow_ms


def aid_ini_payload(lat, lon, alt_m, pos_acc_cm, t_acc_ms, when, leap_seconds):
    """48-byte UBX-AID-INI payload. Altitude is meters; accuracies are cm and ms."""
    week, tow_ms = gps_week_tow_ms(when, leap_seconds)
    return struct.pack(
        "<lllLHHLlLLlLL",
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        int(round(alt_m * 100)),  # cm
        pos_acc_cm,
        0,  # tmCfg: no time pulse
        week,
        tow_ms,
        0,  # towNs
        t_acc_ms,
        0,  # tAccNs
        0,  # clock drift unknown
        0,
        FLAG_POS | FLAG_TIME | FLAG_LLA,
    )


def mga_pos_payload(lat, lon, alt_m, pos_acc_cm):
    """20-byte UBX-MGA-INI-POS_LLH. Altitude and accuracy are centimeters."""
    return struct.pack(
        "<BBHiiiI",
        0x01,  # type: lat/lon/height
        0x00,  # version
        0,  # reserved
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        int(round(alt_m * 100)),
        pos_acc_cm,
    )


def mga_time_payload(when, leap_seconds, t_acc_ms):
    """24-byte UBX-MGA-INI-TIME_UTC. Accuracy is split into seconds and nanoseconds."""
    if not 0 <= leap_seconds <= 127:
        raise ValueError("leap seconds must fit in a signed byte, or be omitted")
    t_acc_ns_total = int(round(t_acc_ms * 1e6))
    t_acc_s, t_acc_ns = divmod(t_acc_ns_total, 1_000_000_000)
    return struct.pack(
        "<BBBbHBBBBBBIHHI",
        0x10,  # type: UTC
        0x00,  # version
        MGA_TIME_ON_RECEIPT,
        leap_seconds,
        when.year,
        when.month,
        when.day,
        when.hour,
        when.minute,
        when.second,
        MGA_TIME_TRUSTED,
        when.microsecond * 1000,
        t_acc_s,
        0,  # reserved
        t_acc_ns,
    )


def hex_payload(payload):
    return ",".join(f"{b:02x}" for b in payload)


def send_ubx(device, protver, cls, mid, payload):
    """Send one raw UBX message. Return (returncode, combined output)."""
    cmd = gpsdev.ubxtool_cmd(
        device,
        protver,
        "-v",
        "1",
        "-w",
        "2",
        "-c",
        f"{cls:02x},{mid:02x},{hex_payload(payload)}",
    )
    proc = subprocess.run(cmd, check=False, capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr


def accepted(protocol, text):
    """True when the receiver did not reject the hint."""
    if "UBX-ACK-NAK" in text:
        return False
    if protocol != "mga":
        return True
    # UBX-MGA-ACK infoCode 0 is acceptance. Any other code is a reject.
    codes = []
    marker = "infoCode "
    for line in text.splitlines():
        start = line.find(marker)
        if start < 0:
            continue
        number = line[start + len(marker):].split()[0].rstrip(",")
        if number.isdigit():
            codes.append(int(number))
    if not codes:
        return False
    return all(code == 0 for code in codes)


def main():
    ap = argparse.ArgumentParser(
        description="Send a u-blox position and time hint via ubxtool."
    )
    ap.add_argument("lat", type=float, help="latitude, degrees")
    ap.add_argument("lon", type=float, help="longitude, degrees")
    ap.add_argument(
        "--device",
        default="/dev/ttyACM0",
        help="gpsd device (ttyACM0, ttyACM1, or a /dev path; default ttyACM0)",
    )
    ap.add_argument(
        "--alt",
        type=float,
        default=180.0,
        help="height above ellipsoid, meters (default 180)",
    )
    ap.add_argument(
        "--pos-acc",
        type=float,
        default=100.0,
        help="position accuracy, meters (default 100)",
    )
    ap.add_argument(
        "--time-acc",
        type=float,
        default=10.0,
        help="time accuracy, seconds (default 10)",
    )
    ap.add_argument(
        "--leap",
        type=int,
        default=18,
        help="GPS-UTC leap seconds (default 18)",
    )
    ap.add_argument(
        "--protver",
        default=None,
        help="ubxtool protocol version (default: the version gpsd reports)",
    )
    args = ap.parse_args()

    if not -90.0 <= args.lat <= 90.0 or not -180.0 <= args.lon <= 180.0:
        sys.exit("lat/lon out of range")
    if args.pos_acc <= 0 or args.time_acc < 0 or args.alt < -1000:
        sys.exit("accuracy must be positive and altitude must be above -1000 m")

    device = gpsdev.canonical_device(args.device)
    info = gpsdev.describe(device)
    protver = float(args.protver) if args.protver is not None else info["protver"]
    protocol = gpsdev.protocol_for(protver)

    now = datetime.now(timezone.utc)
    pos_acc_cm = int(round(args.pos_acc * 100))
    t_acc_ms = int(round(args.time_acc * 1000))
    print(
        f"{protocol.upper()} {device} protver {gpsdev.protver_arg(protver)} "
        f"{now.strftime('%Y-%m-%d %H:%M:%S')}Z "
        f"lat {args.lat:.7f} lon {args.lon:.7f} alt {args.alt:.1f} m "
        f"pos ±{args.pos_acc:.0f} m time ±{args.time_acc:.0f} s "
        f"leap {args.leap}",
        file=sys.stderr,
    )

    if protocol == "mga" and not gpsdev.enable_mga_ack(device, protver):
        sys.exit(f"{device} did not enable aiding acknowledgements")

    if protocol == "aid":
        payload = aid_ini_payload(
            args.lat, args.lon, args.alt, pos_acc_cm, t_acc_ms, now, args.leap
        )
        if len(payload) != 48:
            sys.exit(f"payload length {len(payload)}, expected 48")
        messages = [(0x0B, 0x01, payload)]
    else:
        pos = mga_pos_payload(args.lat, args.lon, args.alt, pos_acc_cm)
        clock = mga_time_payload(now, args.leap, t_acc_ms)
        if len(pos) != 20 or len(clock) != 24:
            sys.exit(f"MGA payload lengths {len(pos)} and {len(clock)}")
        # Time before position. A position hint can be ignored until time is known.
        messages = [(0x13, 0x40, clock), (0x13, 0x40, pos)]

    failed = False
    for cls, mid, payload in messages:
        code, text = send_ubx(device, protver, cls, mid, payload)
        sys.stderr.write(text)
        if code != 0 or not accepted(protocol, text):
            failed = True
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
