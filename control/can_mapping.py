"""Resolve CAN interface names from USB serial numbers.

The physical USB-CAN adapters can enumerate in any order, so the kernel
interface names (can0, can1, ...) are unpredictable across reboots or
replugs. This module maps each arm to its adapter's USB serial number,
which is stable regardless of enumeration order.
"""

import os
import subprocess

ARM_SERIAL = {
    "leader_left":    "001B00523630501120353355",
    "leader_right":   "004500533630501120353355",
    "follower_right": "003100553630501120353355",
    "follower_left":  "004900303945501620303651",
}

SERIAL_TO_ARM = {v: k for k, v in ARM_SERIAL.items()}


def get_serial(iface: str) -> str:
    """Get the USB serial number for a CAN interface."""
    try:
        devpath = os.path.realpath(f"/sys/class/net/{iface}/device/../")
        with open(f"{devpath}/serial") as f:
            return f.read().strip()
    except Exception:
        return ""


def find_can_interfaces() -> list[str]:
    """Find all CAN interfaces from /sys/class/net/."""
    ifaces = []
    try:
        for name in sorted(os.listdir("/sys/class/net")):
            if name.startswith("can"):
                ifaces.append(name)
    except OSError:
        pass
    return ifaces


def build_interface_map() -> dict[str, str]:
    """Build mapping from arm name to current CAN interface name.

    Returns dict like {"leader_left": "can2", "follower_right": "can0", ...}
    """
    mapping = {}
    for iface in find_can_interfaces():
        serial = get_serial(iface)
        arm_name = SERIAL_TO_ARM.get(serial)
        if arm_name:
            mapping[arm_name] = iface
    return mapping


def ensure_up(iface: str) -> bool:
    """Bring a CAN interface up at 1Mbit if it's down. Returns True if up."""
    try:
        with open(f"/sys/class/net/{iface}/operstate") as f:
            state = f.read().strip()
        if state in ("up", "unknown"):
            return True
    except OSError:
        pass

    try:
        subprocess.run(
            ["ip", "link", "set", iface, "down"],
            check=False, capture_output=True,
        )
        subprocess.run(
            ["ip", "link", "set", iface, "up", "type", "can", "bitrate", "1000000"],
            check=True, capture_output=True,
        )
        return True
    except Exception:
        return False


def resolve_arm(arm_name: str) -> str | None:
    """Resolve an arm name to its current CAN interface, bringing it up if needed.

    Usage:
        iface = resolve_arm("leader_left")  # returns e.g. "can2"
    """
    mapping = build_interface_map()
    iface = mapping.get(arm_name)
    if iface and ensure_up(iface):
        return iface
    return None


def print_mapping():
    """Print the current arm-to-interface mapping."""
    mapping = build_interface_map()
    for arm_name in ARM_SERIAL:
        iface = mapping.get(arm_name, "NOT FOUND")
        serial = ARM_SERIAL[arm_name]
        print(f"  {arm_name:20s} -> {iface:6s}  (serial: {serial})")
