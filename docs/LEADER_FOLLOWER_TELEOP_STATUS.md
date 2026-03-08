# OpenArm Leader-Follower Teleoperation -- Status & Notes

**Date**: March 2026
**Repos**:
- `openarm_teleop`: https://github.com/Robokan/openarm_teleop.git (commit `2e1905e`)
- `openarm_can`: `/home/evaughan/sparkpack/openarm_can` (local changes, NOT committed/pushed)

---

## 1. Hardware Setup

### Arms & CAN Interface Mapping

| CAN Interface | Arm           | Role     |
|---------------|---------------|----------|
| `can0`        | Left Leader   | Leader   |
| `can1`        | Right Leader  | Leader   |
| `can2`        | Right Follower| Follower |
| `can3`        | Left Follower | Follower |

### Motor Configuration (per arm)

- 7 arm motors: DM8009 x2 (shoulder), DM4340 x2 (upper arm/elbow), DM4310 x3 (wrist)
- 1 gripper motor: DM4310
- Send CAN IDs: `0x01`-`0x07` (arm), `0x08` (gripper)
- Recv CAN IDs: `0x11`-`0x17` (arm), `0x18` (gripper)

### CAN Bus Setup

The CANable 2.0 (gs_usb) adapters do **NOT** have CAN-FD firmware. All code must use
**CAN 2.0** mode (`enable_fd = false`).

```bash
# Bring up CAN interfaces (run at boot or before use)
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000
```

### URDF Generation

The URDF is generated from xacro using a custom virtual environment:

```bash
# Uses xacro from ~/xacro_venv
~/.local/bin/xacro /home/evaughan/sparkpack/openarm_description/urdf/openarm_v10_bimanual.xacro \
    > /tmp/openarm_urdf_gen/v10_test.urdf
```

### Emergency Motor Disable

If processes are killed with `kill -9` or crash, motors stay enabled. Disable manually:

```bash
for iface in can0 can1 can2 can3; do
  for motor_id in 001 002 003 004 005 006 007 008; do
    cansend $iface ${motor_id}#FFFFFFFFFFFFFFFD 2>/dev/null
  done
done
```

---

## 2. What Works

### Gravity Compensation (single arm)

The standalone `gravity_comp` binary provides working gravity compensation on any single arm.
The arm feels weightless/light and can be moved freely by hand.

```bash
# Build
cd /home/evaughan/sparkpack/openarm_teleop/build
ninja gravity_comp

# Run (example: right leader arm on can1)
./build/gravity_comp right_arm can1 /tmp/openarm_urdf_gen/v10_test.urdf
```

Key parameters that make gravity comp work:
- MIT control with `kp=0, kd=0, torque=gravity_torque` (pure torque mode)
- `MITParam{0, 0, 0, 0, t}` where `t` is the KDL-computed gravity torque
- 100 Hz control loop
- 5ms sleep before `recv_all(10000)` after sending commands

### Anvil Python Tools (reference implementation)

The Anvil `openarm` Python package works correctly for gravity compensation and motor control.
It can be used as a reference to verify hardware functionality:

```bash
# Gravity comp via Python (works perfectly)
cd /home/evaughan/sparkpack/openarm_can
python -m openarm.damiao.gravity --port can1:right

# Refresh/test individual motors
python -m openarm.damiao refresh --port can1 --motor-id 1
```

### Motor Init / Enable

Sequential motor enabling with delays works reliably:

```cpp
// In openarm_init.cpp - enable motors one at a time with 50ms gaps
for (int i = 0; i < openarm->get_arm().get_motors().size(); ++i) {
    openarm->get_arm().enable_one(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    openarm->recv_all(5000);
}
```

This was necessary because the original `enable_all()` sent all enable commands
back-to-back, overflowing the CAN TX buffer and leaving some motors in a bad state.

---

## 3. What Does NOT Work (Blocking Issue)

### recv_all Does Not Update Motor Positions in the Main Loop

**This is THE critical bug preventing teleoperation.**

**Symptom**: After initialization, `recv_all()` never reads any CAN frames in the main
control loop. Motor positions read from `get_motors()[i].get_position()` remain frozen
at their initial values from the enable phase. The gravity compensation *feels* correct
because the initial positions are close enough for small movements, but position feedback
is completely open-loop.

**Impact**: The follower arm never moves because it reads leader positions from shared
state (`robot_state_`), which never updates.

**Diagnostic evidence** (from `can_socket_test.cpp`):

```
=== Test 2: Library OpenArm recv_all ===
After init, motor0 pos = -0.0104906
Sent mit_control_all (7 motors)
Slept 50ms
recv_all(50000): motor0 before=-0.0104906 after=-0.0104906 changed=NO

=== Test 3: Raw socket after library init ===
Raw socket2 select() = 1        <-- A SEPARATE raw socket CAN read responses!
Raw read() = 16 bytes, can_id=0x11
Library recv_all again: before=-0.0104906 after=-0.0104906 changed=NO
```

Also confirmed by running `candump can1` in parallel with the gravity_comp: `candump`
sees all response frames (0x11-0x17) arriving every cycle, but the library's socket
does not.

**What we know**:
- Motor responses ARE arriving on the CAN bus (confirmed via `candump`)
- A separately-created raw CAN socket CAN read them
- The library's own socket CANNOT read them after initialization
- The init phase `recv_all(5000)` DOES work (initial positions are correct)
- Even with 50ms sleep and 50ms first_timeout, zero frames are read
- Both `mit_control_all` and `refresh_all` responses are missed

**Where to investigate**:
The issue is in the `openarm_can` library, specifically in how the CAN socket behaves
after initialization. The socket is created once in the `OpenArm` constructor and used
for both sending and receiving. Something happens between init and the main loop that
breaks receive.

Possible causes to investigate:
1. The `CAN_RAW_LOOPBACK = 0` change may have an unexpected side effect on some kernels
   -- try reverting it
2. The `SO_RCVTIMEO = 100us` set in `can_socket.cpp` may interact badly with `select()`
3. There may be a CAN filter being implicitly set somewhere
4. The socket fd may be getting corrupted or duplicated

**Files involved**:
- `openarm_can/src/openarm/canbus/can_socket.cpp` -- socket creation/config
- `openarm_can/src/openarm/can/socket/openarm.cpp` -- `recv_all()` implementation
- `openarm_can/src/openarm/canbus/can_device_collection.cpp` -- frame dispatch
- `openarm_can/src/openarm/damiao_motor/dm_motor_device.cpp` -- frame callback

---

## 4. Changes Made to `openarm_can` (uncommitted, local only)

These changes exist in `/home/evaughan/sparkpack/openarm_can` but are NOT pushed.
They are installed to `/home/evaughan/.local/lib/libopenarm_can.a` via `sudo ninja install`.

### 4a. Disable CAN loopback (`can_socket.cpp`)

```cpp
// Added after the CAN-FD setsockopt block:
int loopback = 0;
setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
```

**Purpose**: Prevent transmitted frames from echoing back into the receive buffer.

### 4b. Increase subsequent recv timeout (`openarm.cpp`)

```cpp
// Changed from timeout_us = 0 to:
constexpr int subsequent_timeout_us = 500;
// ... in the while loop:
timeout_us = subsequent_timeout_us;
```

**Purpose**: Give time for subsequent motor responses to arrive between reads.

### 4c. Add `enable_one()` method (`dm_motor_device_collection.cpp/.hpp`)

```cpp
void DMDeviceCollection::enable_one(int i) {
    auto dm_device = get_dm_devices().at(i);
    auto& motor = dm_device->get_motor();
    CANPacket enable_packet = CanPacketEncoder::create_enable_command(motor);
    send_command_to_device(dm_device, enable_packet);
}
```

**Purpose**: Enable a single motor (used for sequential init with delays).

### Rebuilding the library

```bash
cd /home/evaughan/sparkpack/openarm_can/build
ninja
sudo ninja install  # Installs to ~/.local/lib/libopenarm_can.a
```

Then rebuild teleop:
```bash
cd /home/evaughan/sparkpack/openarm_teleop/build
ninja
```

---

## 5. Changes Made to `openarm_teleop` (committed to fork)

### 5a. `src/openarm_constants.hpp`
- `FREQUENCY` changed from `200.0` to `100.0`

### 5b. `src/openarm_port/openarm_init.cpp`
- Replaced `enable_all()` with sequential `enable_one(i)` + `sleep(50ms)` + `recv_all(5000)` loop

### 5c. `control/gravity_compasation.cpp`
- Rewrote to use command-line args: `<arm_side> <can_interface> <urdf_path>`
- Gravity torque formula: `MITParam{0, 0, 0, 0, t}` (pure torque, no position/velocity)
- Control loop: `mit_control_all` -> `sleep(5ms)` -> `recv_all(10000)` -> `sleep_until(next_tick)`
- Explicit `enable_fd = false` in init call

### 5d. `src/controller/control.cpp`
- **Leader** `unilateral_step`: gravity formula changed to `effort = gravity[i]` (no negation, no scaling)
- **Leader** MIT params: `kp=0.0, kd=0.0` (pure torque, matching gravity_comp)
- **Follower** `unilateral_step`: reads position references from shared state, uses Kp/Kd from YAML
- All branches: `sleep(5ms)` before `recv_all(10000)`

### 5e. `control/openarm_unilateral_control.cpp`
- Removed `AdjustPosition` calls (arms start from current position)
- Added debug output in AdminThread (prints leader positions every 100 cycles)
- Accepts CLI args: `<leader_urdf> <follower_urdf> [arm_side] [leader_can] [follower_can]`

### 5f. `control/can_socket_test.cpp` (new file)
- Standalone diagnostic that tests raw CAN socket reads vs library recv_all
- Proves the library socket is broken while raw sockets work fine

---

## 6. How to Launch (once recv_all is fixed)

### Gravity Compensation (single arm test)

```bash
# Disable all motors first
for iface in can0 can1 can2 can3; do
  for mid in 001 002 003 004 005 006 007 008; do
    cansend $iface ${mid}#FFFFFFFFFFFFFFFD 2>/dev/null
  done
done

# Run gravity comp on right leader
cd /home/evaughan/sparkpack/openarm_teleop
./build/gravity_comp right_arm can1 /tmp/openarm_urdf_gen/v10_test.urdf
```

### Unilateral Teleoperation (leader -> follower)

```bash
# Right arm pair: leader=can1, follower=can2
./build/unilateral_control \
  /tmp/openarm_urdf_gen/v10_test.urdf \
  /tmp/openarm_urdf_gen/v10_test.urdf \
  right_arm can1 can2

# Left arm pair: leader=can0, follower=can3
./build/unilateral_control \
  /tmp/openarm_urdf_gen/v10_test.urdf \
  /tmp/openarm_urdf_gen/v10_test.urdf \
  left_arm can0 can3
```

### CAN Socket Diagnostic

```bash
./build/can_socket_test
```

---

## 7. Critical Gotchas

1. **Zombie processes**: Always check for lingering processes before starting new ones.
   Multiple processes on the same CAN interface steal each other's frames.
   ```bash
   ps aux | grep -E "gravity_comp|unilateral|bilateral|openarm" | grep -v grep
   ```

2. **Anvil Python gravity comp**: If you run the Anvil Python tools, they stay running
   in the background. Kill them before running C++ code on the same CAN interface.

3. **Motor LEDs**: If motor LEDs are red, the motor is in an error state. Power cycle
   or send the disable command then re-enable.

4. **CAN-FD vs CAN 2.0**: The gravity_comp explicitly passes `enable_fd = false`. The
   unilateral_control uses the default (also `false`). Never set `enable_fd = true`
   unless the CANable firmware is updated.

5. **Build order**: If you change `openarm_can`, you must rebuild AND install it
   (`sudo ninja install`), then rebuild `openarm_teleop`.

---

## 8. Next Steps to Get Teleoperation Working

1. **Fix the recv_all bug** in `openarm_can`. This is the single blocker. Suggested approach:
   - Revert the `CAN_RAW_LOOPBACK = 0` change and test
   - Add debug logging inside `recv_all` to see if `is_data_available()` returns false
     or `read_can_frame()` returns false
   - Compare the socket fd used during init vs main loop (ensure it's the same)
   - Check if `SO_RCVTIMEO = 100us` is causing issues (try removing it or increasing it)

2. **Once positions update**, the follower should immediately start tracking because
   the AdminThread already copies leader responses to follower references.

3. **Run both arm pairs** simultaneously for full bimanual teleoperation.

---

## 9. Reference: Anvil Python Implementation

The Anvil Python code in `openarm_can/openarm/damiao/gravity.py` works correctly. Key
differences from the C++ code:

- Uses `asyncio.gather` to send commands to all motors in parallel
- `bus.recv(master_id, timeout=0.1)` explicitly filters by response CAN ID
- Filters out `is_error_frame` and `not msg.is_rx` (echo frames)
- Queues non-matching messages in `self.lookup` for later retrieval
- Uses `python-can` library which handles socket management differently

The Python implementation proves the hardware is fully functional. The bug is purely
in the C++ `openarm_can` library's socket handling.
