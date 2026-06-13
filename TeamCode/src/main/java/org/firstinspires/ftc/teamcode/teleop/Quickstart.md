# TeleOp Operations Guide 🎮

**Package:** `org.firstinspires.ftc.teamcode.teleop`  
**Last Updated:** May 2026  
**Main Class:** `Teleop.`

---

## 🚀 Quickstart

This is the **main competition TeleOp**. It's a fully automated state-based system that handles intake→shoot transitions, auto-fire sequences, and odometry corrections without requiring complex manual coordination.

**Key Features:**
- ✅ Automatic mode switching (intake ↔ shoot)
- ✅ Auto-fire with pattern-based sequencing
- ✅ Limelight heading auto-correction
- ✅ Non-blocking rescan for color misreads
- ✅ Turret lock for manual aiming override

---

## 🕹️ Controls Reference

### **Gamepad 1 (Driver)**

| Button | Function | Notes |
|--------|----------|-------|
| **Left Stick Y/X** | Field-centric drive | Forward/back and strafe |
| **Right Stick X** | Rotation | Scaled by `getTurnSpeed()` |
| **Triangle** | Intake toggle / Rescan | See [Intake State Machine](#intake-state-machine) |
| **Circle** | Emergency stop | Kills intake, flywheels, resets all states |
| **Square** | Flywheel cycle | OFF → SPIN UP → READY → FIRE (see [Flywheel State Machine](#flywheel-state-machine)) |
| **Cross** | Auto-shoot all balls | Fires all balls in spindexer using pattern order |
| **Left Trigger (>0.3)** | Reverse intake | Unjam balls from intake |
| **Right Trigger (>0.3)** | Toggle turret lock | Locks turret at `TURRET_PARKED_ANGLE` (123.5°) |
| **Left/Right Bumper** | Manual spindexer rotate | For debug/jam clearing |
| **DPad Up** | Reset odometry to corner | Use when flush against wall to correct drift |
| **DPad Down** | Turret offset correction | Minimal odometry change, heading only |
| **DPad Left** | Force intake mode | Useful if mode switching gets stuck |
| **DPad Right** | Force shoot mode | Useful if mode switching gets stuck |

---

## 🎯 State Machines

### Flywheel State Machine

```
OFF (0) 
  ↓ (Square press)
SPINNING UP (1) 
  ↓ (velocity ≥ target - threshold)
  [RUMBLE - 1 blip]
READY (2) 
  ↓ (Square press or Cross for auto-shoot)
Fire → back to SPINNING (1)
```

| State | RPM Target | Action |
|-------|-----------|--------|
| 0 - OFF | 0 | Flywheel idle. |
| 1 - SPINNING | From shooter tables | Ramping up. Rumbles when ready. |
| 2 - READY | Achieved | Can fire. Press Square to fire one ball. |

**Settling Logic:**
```
if (flywheelState == 1
        && Turret.INSTANCE.isSettled()  // Turret on-target AND flywheel at speed
    && !rumbled) {
        gamepad1.rumbleBlips(1);  // 1 blip = ready to fire
flywheelState = 2;
        }
```

---

### Intake State Machine

**Triangle Button Behavior:**

1. **First Press (Intake OFF):** Start intake → move to first free slot.
2. **Second Press (Intake ON):** Non-blocking rescan of all 3 slots.
3. **Auto-transition:** When spindexer fills → Switch to SHOOT mode automatically.

```
IDLE
  ↓ (Triangle)
RUNNING + DWELL (0.3s per slot)
  ↓ (Color sensor stamps slot)
  ↓ (All 3 full?)
YES → enterShootMode()
NO → Move to next free slot
  ↓ (Triangle again)
RESCAN (cycle through all 3 slots, re-stamp colors)
  ↓ (Resume intake at next free slot)
```

---

### Transfer Flick State Machine

Handles the fork mechanism (up/down) with timing and position recovery.

```
IDLE
  ↓ (triggerFlick())
WAIT_UP (0.8s)
  ↓ (Time elapsed)
WAIT_DOWN (1.5s)
  ↓ (Time elapsed)
  [Mark slot EMPTY, toggle position type for settling]
IDLE
```

**Key Detail:** After the flick, the code toggles `PositionType` (INTAKE → SHOOT → INTAKE) to ensure the spindexer servo centers correctly on the next ball.

```
int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
```

---

### Auto-Shoot State Machine

Sequences multiple shots based on pattern detection. Uses distance sensor to confirm ball exit.

```
IDLE
  ↓ (Cross pressed, flywheelState == 2)
CONFIRM (Waiting for distance sensor to clear)
  ↓ (Ball detected as clear?)
YES → NEXT_BALL
NO → (Timeout after 1.5s) → REFIRE_WAIT

NEXT_BALL
  ↓ (Spindexer empty?)
YES → IDLE (all balls shot)
NO → (Turret settled + flick ready?) → CONFIRM (fire next)

REFIRE_WAIT
  ↓ (0.5s delay, turret settled, forks down)
CONFIRM (attempt shot again)
```

**Pattern-Based Firing Order:**
```
if (MatchPattern.isLocked()) {
        switch (MatchPattern.getPattern()) {
        case GPP: order = [GREEN, PURPLE, PURPLE]; break;
        case PGP: order = [PURPLE, GREEN, PURPLE]; break;
        case PPG: order = [PURPLE, PURPLE, GREEN]; break;
        }
        // Fires in this exact order
        }
```

---

## 🎯 Turret Aiming System

### Odometry-Based Aiming (`aimAtGoal`)

When in **SHOOT mode**, the turret automatically tracks the goal using Pinpoint odometry:

```
angleToGoal = atan2(goalY - robotY, goalX - robotX)  // Field frame
turretAngle = angleToGoal - robotHeading              // Robot-relative
setToAngle(turretAngle)
```

**Goal Coordinates (0–144 field system):**
- Blue: (0, 144)
- Red: (144, 144)

### Limelight Auto-Relocalize (NEW)

When the robot **stops** and **can see the goal tag**, Limelight automatically corrects heading drift:

```
if (Limelight.INSTANCE.hasTarget(Limelight.BLUE_GOAL_ID)
    && Math.abs(gamepad1.left_stick_y) < 0.1
        && Math.abs(gamepad1.left_stick_x) < 0.1) {

Double headingToGoal = Limelight.INSTANCE.getHeadingToTag(...);
        Pinpoint.INSTANCE.relocalizeHeadingFromLimelight(headingToGoal);
}
```

**Why this works:**
- Heading is the main source of aiming error
- When you stop, Limelight corrects it passively
- Minimizes need to reset odometry in the corner
- Still preserves X/Y position from encoders

### Turret Lock (Right Trigger)

Press **Right Trigger** to toggle turret lock. When locked, the turret points at `TURRET_PARKED_ANGLE` (123.5°) instead of tracking the goal. Useful if vision/odometry is unreliable.

```
if (!turretLock) {
        Turret.INSTANCE.aimAtGoal(alliance, goalId);
} else {
        Turret.INSTANCE.setToAngle(TURRET_PARKED_ANGLE);
}
```

---

## 📊 Lookup Tables (Shooter Tables)

The turret uses **bilinear interpolation** to automatically select:
- **Flywheel RPM** based on distance from goal
- **Hood angle** (servo position 0.0–1.0) based on distance

These are pre-tuned points loaded from `ShooterTables.`:

```
targetVelo = Turret.INSTANCE.distanceToVelocity(px, py, alliance);
targetHood = Turret.INSTANCE.distanceToPosition(px, py, alliance);
```

**Manual Overrides (Dashboard):**
- `HOOD_OVERRIDE`: Set to 0.0–1.0 to manually set hood. Default -1 (use table).
- `VELO_OVERRIDE`: Set to >0 to manually cap flywheel RPM. Default -1 (use table).

---

## ⚙️ Tunable Constants

| Constant | Default | Purpose |
|----------|---------|---------|
| `TURRET_PARKED_ANGLE` | 123.5° | Angle when turret is locked or in intake mode. |
| `INTAKE_DWELL_SEC` | 0.3s | Time to wait before reading color sensor. |
| `FLICK_UP_SEC` | 0.8s | Time transfer forks stay raised. |
| `FLICK_DOWN_SEC` | 1.5s | Time to wait after forks return before next action. |
| `SHOT_TIMEOUT_SEC` | 1.5s | Max wait for distance sensor before refire. |
| `REFIRE_DELAY_SEC` | 0.5s | Pause between failed shots. |
| `RESCAN_MOVE_SEC` | 0.15s | Time to settle servo before reading color. |

---

## 🔄 Mode Switching

The robot has two modes:

### **INTAKE Mode**
- Intake motor spinning
- Spindexer rotating to free slots
- Turret parked at `TURRET_PARKED_ANGLE`
- Flywheel off
- **Exit:** Auto-switches to SHOOT when all 3 slots full

### **SHOOT Mode**
- Intake idle
- Spindexer rotating to filled slots
- Turret tracking goal (odometry + Limelight)
- Flywheel spinning up
- **Exit:** Manual (Triangle to switch back) or when spindexer empties

**Manual Mode Force:**
- **DPad Left:** Force INTAKE mode
- **DPad Right:** Force SHOOT mode

---

## ⚡ Initialization Flow

```
init()
  └─ Init all subsystems (Drivetrain, Intake, Spindexer, Transfer, Turret, Pinpoint, Limelight)
  └─ Transfer forks DOWN (aggressive)
  └─ Spindexer to INTAKE mode

init_loop()
  └─ Wait for alliance selection (DPad Up = Blue, Down = Red)
  └─ Detect field pattern (obelisk motif)

start()
  └─ Park turret at TURRET_PARKED_ANGLE
  └─ Set initial odometry pose (based on alliance)
  └─ Pre-match scan: Rotate through all 3 slots, read colors
  └─ If full → enterShootMode(); else enterIntakeMode();

loop() [repeated every ~20ms]
  └─ Read inputs + process state machines
  └─ Update Pinpoint odometry
  └─ Auto-aim turret (if in SHOOT mode)
  └─ Auto-relocalize heading from Limelight (if stopped + sees goal)
  └─ Drive robot
  └─ Update telemetry
```

---

## 🐛 Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Robot won't switch to SHOOT | Spindexer not detected as full | Manual: Press DPad Right |
| Color sensor keeps misreading | Lighting/sensor angles | Press Triangle to rescan. Adjust HSV ranges in Spindexer. |
| Turret oscillates wildly | PD tuning or bad odometry | Increase `turretKd` or enable turret lock (Right Trigger) |
| Odometry drifts | Encoder slip | Reset to corner (DPad Up) or wait for Limelight auto-correct |
| Forks jam spindexer | Friction on descent | Already using `transferDownAggressive()` — check mechanical alignment |
| Auto-shoot fires wrong ball | Pattern not locked | Check obelisk detection in init_loop. Manually correct with bumpers. |

---

## 🎓 Advanced Tips

1. **Pre-match Scan:** The robot scans all 3 slots at start. Pre-load balls in **same order** you want to shoot to avoid confusion.

2. **Rescan Usage:** If you load extra balls mid-match, press Triangle (while intake running) to rescan and update the internal color map.

3. **Odometry Drift:** Limelight auto-corrects heading passively when you stop near the goal. This usually keeps aiming accurate for the entire match—reset to corner only if you drift badly.

4. **Manual Override:** Use `HOOD_OVERRIDE` and `VELO_OVERRIDE` on FTC Dashboard during testing to dial in shot distance without redeploying.

5. **Turret Lock:** If odometry is unreliable in a specific area, press Right Trigger to lock turret, then manually aim and fire using hood/velocity overrides.

---

## 📞 Support

For detailed subsystem info, see `Subsystems_README.md`.  
For tuning info, see `Tuning_README.md`.