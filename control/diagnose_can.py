"""Diagnostic: detect motors on all CAN interfaces and map to arms via USB serial."""

import sys

import can

from can_mapping import SERIAL_TO_ARM, ensure_up, find_can_interfaces, get_serial
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.damiao.detect import detect_motors


def main():
    ifaces = find_can_interfaces()

    if not ifaces:
        print("No CAN interfaces found!")
        sys.exit(1)

    all_ok = True

    for iface in ifaces:
        serial = get_serial(iface)
        arm_name = SERIAL_TO_ARM.get(serial, "UNKNOWN ARM")
        print(f"\n--- {iface} -> {arm_name} (serial: {serial}) ---")

        if not ensure_up(iface):
            print(f"  Interface is DOWN - skipping")
            all_ok = False
            continue

        try:
            bus = can.Bus(channel=iface, interface="socketcan")
        except Exception as e:
            print(f"  FAILED to open: {e}")
            all_ok = False
            continue

        try:
            slave_ids = [c.slave_id for c in MOTOR_CONFIGS]
            detected = list(detect_motors(bus, slave_ids, timeout=0.5))
            detected_ids = {d.slave_id for d in detected}

            for c in MOTOR_CONFIGS:
                status = "OK" if c.slave_id in detected_ids else "MISSING"
                if status == "MISSING":
                    all_ok = False
                print(f"  {c.name} (id={c.slave_id}): {status}")

            print(f"  Total: {len(detected)}/8 motors detected")
        finally:
            bus.shutdown()

    print("\n" + ("=" * 40))
    if all_ok:
        print("ALL ARMS HEALTHY")
    else:
        print("ISSUES DETECTED - see above")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
