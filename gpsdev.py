"""Resolve a gpsd device path and which u-blox aiding protocol it speaks.

u-blox 7 (protocol 14) uses UBX-AID. u-blox 8 and 9 (protocol 15 and later)
use UBX-MGA. gpsd must already be talking to the receiver.
"""

import json
import re
import subprocess
import sys

# Short names for the two sticks on this host. Anything else is used as a path.
ALIASES = {
    "0": "/dev/ttyACM0",
    "1": "/dev/ttyACM1",
    "acm0": "/dev/ttyACM0",
    "acm1": "/dev/ttyACM1",
    "gps0": "/dev/ttyACM0",
    "gps1": "/dev/ttyACM1",
    "ttyacm0": "/dev/ttyACM0",
    "ttyacm1": "/dev/ttyACM1",
}


def canonical_device(name):
    """Return the gpsd path for a CLI device name."""
    text = name.strip()
    key = text.lower().removeprefix("/dev/")
    if key in ALIASES:
        return ALIASES[key]
    if text.startswith("/dev/"):
        return text
    if text.startswith("tty"):
        return "/dev/" + text
    return text


def parse_protver(subtype):
    """Protocol version from a gpsd subtype string, or None."""
    match = re.search(r"PROTVER[= ]([0-9]+(?:\.[0-9]+)?)", subtype or "")
    if not match:
        return None
    return float(match.group(1))


def protocol_for(protver):
    """'aid' below protocol 15, 'mga' from u-blox 8 onward."""
    return "mga" if float(protver) >= 15.0 else "aid"


def protver_arg(protver):
    """ubxtool -P text. 14.0 prints as 14, 32.01 stays 32.01."""
    text = f"{float(protver):.2f}"
    return text.rstrip("0").rstrip(".")


def describe(device):
    """gpsd's record for one device: path, protver, protocol, subtype."""
    proc = subprocess.run(
        ["timeout", "5", "gpspipe", "-w", "-n", "8"],
        check=False,
        capture_output=True,
        text=True,
    )
    found = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        if msg.get("class") != "DEVICES":
            continue
        for item in msg.get("devices", []):
            if item.get("path") == device:
                found = item
                break
    if found is None:
        sys.exit(f"{device} is not open in gpsd")
    subtype = found.get("subtype1") or ""
    protver = parse_protver(subtype)
    if protver is None:
        sys.exit(f"gpsd did not report a protocol version for {device}")
    return {
        "path": device,
        "protver": protver,
        "protocol": protocol_for(protver),
        "subtype": found.get("subtype") or "",
        "subtype1": subtype,
    }


def ubxtool_cmd(device, protver, *args):
    """ubxtool command aimed at one gpsd device."""
    return [
        "ubxtool",
        "-P",
        protver_arg(protver),
        "--device",
        device,
        *args,
    ]


def set_mga_ack(device, protver, enabled):
    """Turn aiding acknowledgements on or off in RAM, not in flash.

    u-blox 9 sends UBX-MGA-ACK only when CFG-NAVSPG-ACKAIDING is on.
    gpsd's ubxtool crashes if it tries to decode one at verbosity 2 or
    higher, so a database poll has to run with acknowledgements off.
    Returns True when the receiver accepted the setting.
    """
    proc = subprocess.run(
        ubxtool_cmd(
            device,
            protver,
            "-v",
            "1",
            "-w",
            "1.5",
            "-z",
            f"CFG-NAVSPG-ACKAIDING,{1 if enabled else 0},1",
        ),
        check=False,
        capture_output=True,
        text=True,
    )
    text = proc.stdout + proc.stderr
    if proc.returncode != 0 or "UBX-ACK-NAK" in text:
        sys.stderr.write(text)
        return False
    return True


def enable_mga_ack(device, protver):
    """Ask the receiver to answer aiding messages."""
    return set_mga_ack(device, protver, True)
