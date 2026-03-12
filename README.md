# OpenArm Teleop

Real-time teleoperation system for Enactic OpenArm robotic arms using DaMiao motors over CAN bus. Supports unilateral (leader gravity-compensated, follower tracks) and bilateral (force feedback) control modes with bimanual operation.

## Hardware

- **Arms**: 4x Enactic OpenArm (2 leader, 2 follower)
- **Motors**: 7 arm joints (2x DM8009, 2x DM4340, 3x DM4310) + 1 gripper (DM4310) per arm
- **CAN adapters**: 4x CANable 2.0 (gs_usb, CAN 2.0 only — no CAN FD)
- **CAN bus**: 1 Mbit/s, daisy-chained through each arm

### CAN Interface Mapping

Each CANable adapter is identified by USB serial number, mapped to a logical arm name:

| Serial | Arm | Default canX |
|--------|-----|-------------|
| `001B00523630501120353355` | leader_left | can0 |
| `004500533630501120353355` | leader_right | can1 |
| `003100553630501120353355` | follower_right | can2 |
| `004900303945501620303651` | follower_left | can3 |

The kernel assigns `can0`–`can3` in arbitrary order depending on USB enumeration. The code auto-resolves the correct interface using serial numbers via `src/can_interface_resolver.hpp`. udev rules in `config/99-openarm-can.rules` can also provide stable naming.

### CAN IDs

- Arm motors: send 0x01–0x07, receive 0x11–0x17
- Gripper motor: send 0x08, receive 0x18

## Building

### Dependencies

- [openarm_can](https://github.com/enactic/openarm_can) — CAN communication library
- orocos_kdl — kinematics/dynamics
- kdl_parser — URDF to KDL chain
- Eigen3
- urdfdom / urdfdom_headers
- yaml-cpp

### Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Executables

| Binary | Purpose |
|--------|---------|
| `bilateral_control` | Bilateral teleop with force feedback |
| `unilateral_control` | Unilateral teleop (leader free-floating, follower tracks) |
| `gravity_comp` | Gravity compensation on a single arm |
| `comm_test` | CAN communication test/demo |

## URDF Generation

URDFs are generated from xacro files in the separate `openarm_description` ROS 2 package:

```bash
export AMENT_PREFIX_PATH=~/openarm_ros2_ws/install
XACRO_PATH=~/openarm_ros2_ws/src/openarm_description/urdf/robot/openarm_v10_bimanual.xacro
xacro "$XACRO_PATH" bimanual:=true -o /tmp/openarm_urdf_gen/v10.urdf
```

The URDF contains both left and right arm chains. Each chain starts at `openarm_body_link0` and ends at `openarm_left_hand` or `openarm_right_hand`. The left and right arms are mirrored (different joint frame orientations and axis signs).

## Running

All executables require `sudo` for real-time scheduling (`SCHED_FIFO`) and CAN access.

### Single arm pair

```bash
# Unilateral (leader free-floating, follower tracks)
sudo ./build/unilateral_control /tmp/openarm_urdf_gen/v10.urdf /tmp/openarm_urdf_gen/v10.urdf left_arm

# Bilateral (force feedback)
sudo ./build/bilateral_control /tmp/openarm_urdf_gen/v10.urdf /tmp/openarm_urdf_gen/v10.urdf left_arm

# Gravity compensation only
sudo ./build/gravity_comp left_arm /tmp/openarm_urdf_gen/v10.urdf
```

### Bimanual (two terminals)

```bash
# Terminal 1
sudo ./build/bilateral_control /tmp/openarm_urdf_gen/v10.urdf /tmp/openarm_urdf_gen/v10.urdf left_arm

# Terminal 2
sudo ./build/bilateral_control /tmp/openarm_urdf_gen/v10.urdf /tmp/openarm_urdf_gen/v10.urdf right_arm
```

### Arguments

```
bilateral_control <leader_urdf> <follower_urdf> [arm_side] [leader_can] [follower_can]
unilateral_control <leader_urdf> <follower_urdf> [arm_side] [leader_can] [follower_can]
gravity_comp <arm_side> <urdf_path> [can_interface]
```

- `arm_side`: `left_arm` or `right_arm`
- CAN interfaces are auto-resolved from USB serial numbers if not specified

## Architecture

### Overview

```
┌──────────────────────────────────────────────────────────┐
│                    Main Process                          │
│                                                          │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐ │
│  │ LeaderArm   │  │ FollowerArm  │  │ Admin           │ │
│  │ Thread      │  │ Thread       │  │ Thread          │ │
│  │ (1000 Hz)   │  │ (1000 Hz)    │  │ (1000 Hz)       │ │
│  │             │  │              │  │                 │ │
│  │ bilateral_  │  │ bilateral_   │  │ Exchange refs   │ │
│  │ step() or   │  │ step() or    │  │ between leader  │ │
│  │ unilateral_ │  │ unilateral_  │  │ and follower    │ │
│  │ step()      │  │ step()       │  │ state objects   │ │
│  └──────┬──────┘  └──────┬───────┘  └────────┬────────┘ │
│         │                │                    │          │
│         ▼                ▼                    ▼          │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐ │
│  │ leader_     │  │ follower_    │  │ Reads/writes    │ │
│  │ state       │◄─┤ state        │  │ both state      │ │
│  │ (responses  │  │ (responses   │  │ objects to copy │ │
│  │  references)│  │  references) │  │ positions       │ │
│  └──────┬──────┘  └──────┬───────┘  └─────────────────┘ │
│         │                │                               │
│         ▼                ▼                               │
│  ┌─────────────┐  ┌──────────────┐                      │
│  │ OpenArm     │  │ OpenArm      │                      │
│  │ (CAN bus)   │  │ (CAN bus)    │                      │
│  │ leader canX │  │ follower canX│                      │
│  └─────────────┘  └──────────────┘                      │
└──────────────────────────────────────────────────────────┘
```

### Threading Model

Three `PeriodicTimerThread` instances run at 1000 Hz using `SCHED_FIFO` real-time scheduling and `clock_nanosleep(TIMER_ABSTIME)`:

1. **LeaderArmThread** — runs the leader's control step
2. **FollowerArmThread** — runs the follower's control step
3. **AdminThread** — exchanges position references between leader and follower state objects

The threads share `leader_state` and `follower_state` (`RobotSystemState`) without explicit locking. The AdminThread copies responses from one arm to references of the other.

### Data Flow

Each control tick:

1. **Leader/Follower threads** read motor positions via CAN (`recv_all`), convert motor→joint space, store in `robot_state_.responses`
2. **AdminThread** copies: `leader.response → follower.reference` and `follower.response → leader.reference`
3. **Leader/Follower threads** read `robot_state_.references`, compute gravity/friction, send MIT commands via CAN

## Control Modes

### MIT Control

All motors use DaMiao MIT (Motor Impedance Torque) mode. Each command contains:

```
τ_motor = Kp × (q_ref - q_actual) + Kd × (dq_ref - dq_actual) + τ_ff
```

- `Kp`, `Kd`: position/velocity gains (from YAML config)
- `q_ref`, `dq_ref`: reference position/velocity (from the other arm)
- `τ_ff`: feedforward torque (gravity + friction compensation)

### Unilateral Control

The leader arm is free-floating (gravity compensated) and the follower tracks the leader's position.

**Leader** (`unilateral_step`, `ROLE_LEADER`):
- MIT params: `Kp=0, Kd=0` (pure torque control, no position tracking)
- Feedforward: `gravity × grav_scale + friction × 0.3 + coriolis × 0.1`
- `grav_scale = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.5}` — URDF overestimates mass/inertia, especially J7 (wrist)
- Result: arm floats freely, user moves it by hand

**Follower** (`unilateral_step`, `ROLE_FOLLOWER`):
- MIT params: `Kp, Kd` from `config/follower.yaml`, reference = leader's position
- Feedforward: `gravity × grav_scale + friction` (full gravity comp so Kp doesn't fight gravity)
- `grav_scale = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.75}`
- Result: follower mirrors leader position with gravity assistance

### Bilateral Control

Both arms use position tracking with gravity compensation. The leader feels force feedback from the follower.

**Leader** (`bilateral_step`, `ROLE_LEADER`):
- MIT params: `Kp, Kd` from `config/leader.yaml` (low values: Kp=8/4/3 for light feel), reference = follower's position
- Feedforward: `gravity × grav_scale + friction × 0.3 + coriolis × 0.1` (same as unilateral leader for free-floating feel)
- Result: feels nearly free-floating, gentle force feedback when follower hits resistance

**Follower** (`bilateral_step`, `ROLE_FOLLOWER`):
- MIT params: `Kp, Kd` from `config/follower.yaml` (higher: Kp=30/15/12), reference = leader's position
- Feedforward: `gravity × grav_scale + friction`
- Result: tracks leader tightly with gravity compensation

### Gravity Compensation Scaling

The URDF overestimates arm mass/inertia. Per-joint scaling reduces gravity torques:

```cpp
// bilateral_step (both roles)
const double grav_scale[] = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.75};

// unilateral_step leader only
const double grav_scale[] = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.5};
```

J7 (last wrist joint) needs the most reduction. Bilateral uses 0.75 for J7; unilateral leader uses 0.5 (different because the bilateral Kp provides additional stabilization).

## Gripper Scaling

The leader and follower grippers have different physical travel ranges. An offset and scale maps between them:

```
follower_pos = (leader_pos + 0.06) × 2.58
leader_pos = follower_pos / 2.58 - 0.06
```

- **Offset (0.06)**: compensates for the leader gripper's non-zero position when physically fully closed (~-0.024 rad)
- **Scale (2.58)**: ratio of follower to leader gripper travel (measured: leader fully open = -1.12, follower fully open = -2.90)

### Minimum Gripper Closing Force (bilateral only)

With low Kp, the closing force drops to zero as the gripper approaches the target. A minimum position error of 0.2 rad is enforced so the gripper always has enough force to close:

```cpp
if (ref > actual && ref - actual < min_grip_error)
    ref = actual + min_grip_error;
```

## Elbow Spring (bilateral only)

A soft virtual spring prevents the elbow from locking at full extension. When the elbow (J4, index 3) is within 20° of straight, the reference position is nudged away from extension:

```cpp
constexpr double elbow_zone = 0.349;   // ~20 degrees
constexpr double elbow_blend = 1.05;
if (position < elbow_zone)
    position += (elbow_zone - position) * elbow_blend;
```

This is applied to both leader and follower arm responses in the AdminThread before setting references.

## Startup Sequence

1. **Initialize motors**: set MIT control mode, enable all motors
2. **Read leader position**: `refresh_all()` + `recv_all()` to get leader's current joint positions
3. **Set follower target**: set follower references to leader's current position (with gripper scaling)
4. **Smooth interpolation**: `AdjustPosition()` moves the follower from its current position to the leader's position over 220 steps (~2.2 seconds) using fixed gains (Kp=50, Kd=1.2)
5. **Start control loops**: launch Leader, Follower, and Admin threads at 1000 Hz

The leader arm stays at its current position — no homing or startup motion on the leader.

## Configuration

### `config/leader.yaml`

```yaml
LeaderArmParam:
  Kp: [8.0, 8.0, 8.0, 4.0, 3.0, 3.0, 3.0, 2.0]    # J1-J4 shoulder/elbow, J5-J7 wrist, J8 gripper
  Kd: [1.0, 1.0, 1.0, 1.0, 0.15, 0.15, 0.15, 0.15]
  Fc: [...]   # Coulomb friction magnitude
  k:  [...]   # tanh steepness
  Fv: [...]   # Viscous friction coefficient
  Fo: [...]   # Constant bias torque
```

### `config/follower.yaml`

```yaml
FollowerArmParam:
  Kp: [30.0, 30.0, 30.0, 15.0, 12.0, 15.0, 12.0, 2.0]
  Kd: [4.0, 4.0, 4.0, 4.0, 0.15, 0.15, 0.15, 0.15]
  # same friction params structure
```

### Friction Model

```
τ_friction(ω) = Fo + Fv·ω + Fc·tanh(k·ω)
```

## Key Classes

### `Control` (`src/controller/control.cpp`)

Core control logic. One instance per arm.

- `SetParameter(Kp, Kd, Fc, k, Fv, Fo)` — set gains and friction params
- `bilateral_step()` — one tick of bilateral control (gravity comp + Kp/Kd tracking)
- `unilateral_step()` — one tick of unilateral control (leader: gravity comp only; follower: gravity comp + Kp/Kd tracking)
- `AdjustPosition()` — smooth interpolation to a target position at startup
- `ComputeFriction()` — tanh friction model

### `Dynamics` (`src/controller/dynamics.cpp`)

KDL-based dynamics using URDF.

- `Init()` — parse URDF, build KDL chain
- `GetGravity(positions, output)` — compute gravity torques
- `GetCoriolis(positions, velocities, output)` — compute Coriolis torques
- Uses chain from `openarm_body_link0` to `openarm_{left,right}_hand`

### `RobotSystemState` (`src/robot_state.hpp`)

Holds arm and hand state. Shared between threads.

- `arm_state()` → `RobotState` (7 joints)
- `hand_state()` → `RobotState` (1 joint)
- Each `RobotState` has `responses` (current positions from motors) and `references` (target positions from other arm)

### `OpenArmJointConverter` / `OpenArmJGripperJointConverter` (`src/joint_state_converter.hpp`)

Convert between motor space and joint space. Currently 1:1 mapping (no gear ratios).

- `motor_to_joint(vector<MotorState>)` → `vector<JointState>`
- `joint_to_motor(vector<JointState>)` → `vector<MotorState>`

### `PeriodicTimerThread` (`src/periodic_timer_thread.hpp`)

Abstract base for real-time periodic threads. Uses `SCHED_FIFO` priority 80, `clock_nanosleep`.

- `before_start()`, `on_timer()`, `after_stop()` — override in subclass
- Constructor takes Hz (default 1000)

## CAN Bus Timing

CANable 2.0 at 1 Mbit/s needs ~2 ms for all 8 motors to respond after a command broadcast. The control loop sleeps 2 ms then calls `recv_all(5000)` (5 ms timeout) to read all responses:

```cpp
std::this_thread::sleep_for(std::chrono::milliseconds(2));
openarm_->recv_all(5000);
```

This limits the effective control rate to ~300-370 Hz per arm despite the 1000 Hz timer.

## File Structure

```
openarm_teleop/
├── CMakeLists.txt
├── config/
│   ├── leader.yaml              # Leader arm Kp/Kd/friction params
│   ├── follower.yaml            # Follower arm Kp/Kd/friction params
│   └── 99-openarm-can.rules     # udev rules for stable CAN naming
├── control/
│   ├── openarm_bilateral_control.cpp    # Bilateral teleop executable
│   ├── openarm_unilateral_control.cpp   # Unilateral teleop executable
│   ├── gravity_compasation.cpp          # Gravity comp standalone executable
│   ├── openarm_communication_test.cpp   # CAN communication test
│   ├── can_socket_test.cpp              # Raw CAN socket test
│   ├── leader_follower.py               # Python leader-follower teleop
│   ├── diagnose_can.py                  # CAN diagnostics
│   └── can_mapping.py                   # Python CAN serial mapping
├── src/
│   ├── controller/
│   │   ├── control.cpp / control.hpp    # Control logic (bilateral/unilateral steps)
│   │   ├── dynamics.cpp / dynamics.hpp  # KDL gravity/Coriolis computation
│   │   └── diff.hpp                     # Velocity differentiator
│   ├── openarm_port/
│   │   ├── openarm_init.cpp / .hpp      # Motor initialization
│   │   └── joint_mapper.cpp / .hpp      # Motor↔joint ID mapping
│   ├── robot_state.hpp                  # JointState, RobotState, RobotSystemState
│   ├── joint_state_converter.hpp        # MotorState, motor↔joint conversion
│   ├── periodic_timer_thread.hpp        # Real-time periodic thread base class
│   ├── openarm_constants.hpp            # Motor types, CAN IDs, limits, frequency
│   ├── yamlloader.hpp                   # YAML config loader
│   └── can_interface_resolver.hpp       # USB serial → CAN interface resolution
├── script/
│   ├── bilateral_control.sh             # Launch script for bilateral
│   ├── unilateral_control.sh            # Launch script for unilateral
│   └── gravity_comp.sh                  # Launch script for gravity comp
└── vr_teleop/
    ├── vr_teleop.py                     # VR teleop with camera + controller
    ├── controller_tracker.py            # OpenXR controller tracking
    └── camera_viewer.py                 # Camera viewer
```

## Integration Guide

### Programmatic Control (bypassing the leader arm)

To control the follower arm(s) programmatically without a physical leader, you need to:

1. **Initialize hardware** — use `OpenArmInitializer::initialize_openarm()` to set up the CAN bus and enable motors
2. **Create Control object** — with `ROLE_FOLLOWER`, dynamics, and state objects
3. **Set references** — write target joint positions to `robot_state->arm_state().set_all_references()` and `hand_state().set_all_references()`
4. **Call control step** — `control->unilateral_step()` or `control->bilateral_step()` in a real-time loop

The follower's control step reads references from `robot_state_`, computes gravity compensation, and sends MIT commands with Kp/Kd tracking toward the reference positions.

### Key integration points

- **Joint positions** are in radians, in joint space (after motor-to-joint conversion, currently 1:1)
- **7 arm joints** + **1 gripper joint** per arm
- **Arm joint order**: J1 (shoulder rotation) → J7 (wrist roll)
- **Control frequency**: should be ≥200 Hz for stability; the CAN bus timing (2ms sleep + recv_all) sets the practical minimum
- **References must be set continuously** — the control step reads them every tick
- **Gravity compensation is critical** — without feedforward gravity torques, the Kp must fight gravity alone, causing steady-state position error proportional to gravity_load/Kp
- **Motor limits** are defined in `openarm_constants.hpp` (`POSITION_LIMIT_L`, `POSITION_LIMIT_H`)

### Example: sending a trajectory to the follower

```cpp
#include "openarm_port/openarm_init.hpp"
#include "controller/control.hpp"
#include "controller/dynamics.hpp"
#include "robot_state.hpp"
#include "joint_state_converter.hpp"
#include "openarm_constants.hpp"

// 1. Initialize hardware
auto* openarm = openarm_init::OpenArmInitializer::initialize_openarm("can3", true, false);

// 2. Set up dynamics and state
Dynamics dynamics("path/to/urdf", "openarm_body_link0", "openarm_left_hand");
dynamics.Init();
auto state = std::make_shared<RobotSystemState>(7, 1);  // 7 arm + 1 gripper

// 3. Create control
Control control(openarm, &dynamics, &dynamics, state, 0.001, ROLE_FOLLOWER);
control.SetParameter(kp, kd, fc, k, fv, fo);  // from YAML

// 4. Real-time loop
while (running) {
    // Set target position
    std::vector<JointState> target(7);
    for (int i = 0; i < 7; i++) {
        target[i].position = desired_positions[i];
        target[i].velocity = desired_velocities[i];
    }
    state->arm_state().set_all_references(target);

    // Run one control tick
    control.unilateral_step();

    // Sleep to maintain loop rate
    // (note: unilateral_step already sleeps 2ms + recv_all internally)
}

openarm->disable_all();
```

### Safety notes

- Always start with `AdjustPosition()` to smoothly move to the first target
- The follower's `unilateral_step` includes gravity compensation — don't add your own
- If the arm thrashes, check: (a) CAN cable connections, (b) URDF correctness, (c) all motors responding to `recv_all`
- Use `Ctrl+C` to stop — the signal handler disables motors on exit
- CAN bus cables through the elbow joint can become intermittent when the elbow bends — reseat connectors if wrist motors stop responding
