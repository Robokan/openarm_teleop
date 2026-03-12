"""Leader-follower teleop built on top of Anvil's gravity compensation.

Mirrors the C++ bilateral_control pattern: send all commands, sleep 5ms,
then read all responses. Uses a single shared Bus per CAN interface.

Usage:
    # By arm name (resolves USB serial automatically):
    leader_follower.py --side left
    leader_follower.py --side right

    # Or override CAN interfaces manually:
    leader_follower.py --leader-can can0 --follower-can can3 --side left
"""

import argparse
import asyncio
import signal
import sys
import time

import can

from can_mapping import print_mapping, resolve_arm
from openarm.bus import Bus
from openarm.damiao import ControlMode, Motor
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.damiao.detect import detect_motors
from openarm.damiao.encoding import MitControlParams, decode_motor_state, encode_control_mit
from openarm.damiao.gravity import ArmWithGravity, GravityCompensator

# Position tracking gains (stable for Python loop timing)
KP = [50.0, 50.0, 50.0, 50.0, 14.0, 14.0, 14.0, 8.0]
KD = [3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 0.8]


def setup_arm(can_bus: can.BusABC, position: str) -> ArmWithGravity | None:
    """Set up arm with a single shared Bus to avoid response-stealing."""
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.5))
    detected_ids = {info.slave_id for info in detected}

    missing = [c.name for c in MOTOR_CONFIGS if c.slave_id not in detected_ids]
    if missing:
        print(f"  Missing motors: {missing}")
        return None

    shared_bus = Bus(can_bus)
    motors = []
    for config in MOTOR_CONFIGS:
        motor = Motor(
            shared_bus,
            slave_id=config.slave_id,
            master_id=config.master_id,
            motor_type=config.type,
        )
        motors.append(motor)

    return ArmWithGravity(motors=motors, position=position, can_bus=can_bus)


async def send_then_recv(arm, kp, kd, q, dq, tau):
    """Mirror the C++ pattern: send all commands, sleep 5ms, read all responses.

    This matches:
        openarm_->get_arm().mit_control_all(arm_cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        openarm_->recv_all(10000);
    """
    # Phase 1: send all MIT commands (no waiting for responses)
    for i, motor in enumerate(arm.motors):
        kp_i = kp[i] if isinstance(kp, list) else kp
        kd_i = kd[i] if isinstance(kd, list) else kd
        q_i = q[i] if isinstance(q, list) else q
        dq_i = dq[i] if isinstance(dq, list) else dq
        tau_i = tau[i] if isinstance(tau, list) else tau
        encode_control_mit(
            motor._bus, motor._slave_id, motor._motor_limits,
            MitControlParams(kp=kp_i, kd=kd_i, q=q_i, dq=dq_i, tau=tau_i),
        )

    # Phase 2: wait for motors to process (like C++ 5ms sleep)
    time.sleep(0.005)

    # Phase 3: read all responses
    states = []
    for motor in arm.motors:
        try:
            state = await decode_motor_state(motor._bus, motor._master_id, motor._motor_limits)
            states.append(state)
        except Exception:
            states.append(None)
    return states


def parse_args():
    parser = argparse.ArgumentParser(description="Leader-follower teleop")
    parser.add_argument(
        "--side", choices=["left", "right"], default="left",
        help="Which arm pair to use (default: left)",
    )
    parser.add_argument("--leader-can", help="Override leader CAN interface (e.g. can0)")
    parser.add_argument("--follower-can", help="Override follower CAN interface (e.g. can3)")
    parser.add_argument(
        "--leader-only", action="store_true",
        help="Run leader in gravity comp only (no follower)",
    )
    return parser.parse_args()


def resolve_interfaces(args) -> tuple[str, str | None, str]:
    """Resolve CAN interfaces from args, using USB serial lookup if not overridden."""
    side = args.side

    print("Current CAN interface mapping:")
    print_mapping()
    print()

    if args.leader_can:
        leader_can = args.leader_can
    else:
        leader_can = resolve_arm(f"leader_{side}")
        if not leader_can:
            print(f"ERROR: Could not find CAN adapter for leader_{side}!")
            sys.exit(1)

    if args.leader_only:
        follower_can = None
    elif args.follower_can:
        follower_can = args.follower_can
    else:
        follower_can = resolve_arm(f"follower_{side}")
        if not follower_can:
            print(f"ERROR: Could not find CAN adapter for follower_{side}!")
            sys.exit(1)

    return leader_can, follower_can, side


async def main():
    args = parse_args()
    leader_can, follower_can, side = resolve_interfaces(args)

    keep_running = True

    def on_sigint(sig, frame):
        nonlocal keep_running
        print("\nStopping...")
        keep_running = False

    signal.signal(signal.SIGINT, on_sigint)

    gravity_comp = GravityCompensator()

    leader_bus = can.Bus(channel=leader_can, interface="socketcan")
    follower_bus = can.Bus(channel=follower_can, interface="socketcan") if follower_can else None

    all_buses = [leader_bus] + ([follower_bus] if follower_bus else [])
    leader_arm = None
    follower_arm = None

    try:
        print(f"Setting up leader on {leader_can} ({side})...")
        leader_arm = setup_arm(leader_bus, side)
        if not leader_arm:
            print("Failed to detect leader motors!")
            return

        states = await leader_arm.enable()
        await leader_arm.set_control_mode(ControlMode.MIT)
        for i, state in enumerate(states):
            if state:
                leader_arm.positions[i] = state.position
        print(f"  Leader ready: {[f'{p:.3f}' for p in leader_arm.positions]}")

        if follower_bus:
            print(f"Setting up follower on {follower_can} ({side})...")
            follower_arm = setup_arm(follower_bus, side)
            if not follower_arm:
                print("Failed to detect follower motors! Running leader-only.")

            if follower_arm:
                states = await follower_arm.enable()
                await follower_arm.set_control_mode(ControlMode.MIT)
                for i, state in enumerate(states):
                    if state:
                        follower_arm.positions[i] = state.position
                print(f"  Follower ready: {[f'{p:.3f}' for p in follower_arm.positions]}")

        mode = "Leader-Follower" if follower_arm else "Leader gravity comp only"
        print(f"\n=== {mode} === Press Ctrl+C to stop.")

        tick = 0
        while keep_running:
            t0 = time.monotonic()

            # Leader: gravity comp only
            torques = gravity_comp.compute(leader_arm.positions, leader_arm.position)
            states = await send_then_recv(leader_arm, kp=0, kd=0, q=0, dq=0, tau=torques)
            for i, state in enumerate(states):
                if state:
                    leader_arm.positions[i] = state.position

            # Follower: position tracking toward leader
            if follower_arm:
                states = await send_then_recv(
                    follower_arm, kp=KP, kd=KD, q=leader_arm.positions, dq=0, tau=0,
                )
                for i, state in enumerate(states):
                    if state:
                        follower_arm.positions[i] = state.position

            tick += 1
            ms = (time.monotonic() - t0) * 1000
            if tick % 100 == 0:
                print(f"[{tick}] {ms:.1f}ms")
                if follower_arm:
                    for i in range(len(follower_arm.positions)):
                        lp = leader_arm.positions[i]
                        fp = follower_arm.positions[i]
                        print(f"  J{i+1}: leader={lp:+.3f} follower={fp:+.3f} err={abs(lp-fp):.3f}")

            elapsed = time.monotonic() - t0
            remaining = 0.01 - elapsed
            if remaining > 0:
                time.sleep(remaining)

    finally:
        print("Disabling...")
        if leader_arm:
            for motor in leader_arm.motors:
                try:
                    await motor.disable()
                except Exception:
                    pass
        if follower_arm:
            for motor in follower_arm.motors:
                try:
                    await motor.disable()
                except Exception:
                    pass
        for b in all_buses:
            b.shutdown()
        print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
