# TeleOp Operations Guide 🎮

**Package:** `org.firstinspires.ftc.teamcode.teleop`  
**Last Updated:** June 2026  
**Main Class:** `Teleop.`

---

## 🚀 Quickstart

This is the **main competition TeleOp**. It's a fully automated state-based system that handles intake→shoot transitions, auto-fire sequences, and odometry corrections without requiring complex manual coordination.

**Key Features:**
- ✅ Automatic mode switching (intake ↔ shoot)
- ✅ Auto-fire with pattern-based sequencing
- ✅ Limelight MegaTag1 position relocalization (manual + passive-while-stopped)
- ✅ Non-blocking rescan for color misreads
- ✅ Turret lock for manual aiming override
- ✅ Spindexer settle gating — fork can't fire until the spindexer has actually arrived at its commanded slot

---

## 🕹️ Controls Reference

### **Gamepad 1 (Driver)**

| Button | Function | Notes |
|--------|----------|-------|
| **Left Stick Y/X** | Robot-centric drive | Forward/back and strafe |
| **Right Stick X** | Rotation | Scaled by `getTurnSpeed()` |
| **Triangle** | Intake toggle / misread recovery | See Intake State Machine below |
| **Circle** | Emergency stop | Kills intake, flywheel, resets all states, returns to INTAKE mode |
| **Square** | Flywheel cycle / fire one ball | OFF → SPIN UP → READY → FIRE |
| **Cross** | Auto-shoot all balls (SHOOT mode only) | Fires all balls in spindexer using pattern order |
| **Left Trigger (>0.3)** | Reverse intake | Unjam balls from intake |
| **Right Trigger (>0.3)** | Toggle turret lock | Locks turret at `TURRET_PARKED_ANGLE` (270°) instead of tracking the goal |
| **Left/Right Bumper** | Manual spindexer rotate | For debug/jam clearing |
| **DPad Up** | Reset odometry to alliance corner | Sets position AND heading — use when flush against the wall |
| **DPad Down** | Relocalize X/Y from Limelight (MT1) | Averages 5 vision samples; corrects position only — heading is left untouched |
| **DPad Left** | Force intake mode | Useful if mode switching gets stuck |
| **DPad Right** | Force shoot mode | Useful if mode switching gets stuck |

---

## 🎯 State Machines

### Flywheel State Machine

```
OFF (0) 
  -> (Square press)
SPINNING UP (1) 
  -> (velocity >= target - threshold AND turret on-target)
  [RUMBLE - 1 blip]
READY (2) 
  -> (Square press, if spindexer has settled - see below)
Fire -> back to SPINNING (1)
```

| State | RPM Target | Action |
|-------|-----------|--------|
| 0 - OFF | 0 | Flywheel idle. |
| 1 - SPINNING | From shooter tables | Ramping up. Rumbles once settled. |
| 2 - READY | Achieved | Can fire, if spindexer is also settled on its slot. |

**Settling Logic** (`isSettled()` checks BOTH flywheel speed and turret position):
```
if (flywheelState == 1 && Turret.INSTANCE.isSettled() && !rumbled) {
    gamepad1.rumbleBlips(1);  // 1 blip = flywheel + turret ready
    rumbled = true;
    flywheelState = 2;
}
```

**Spindexer Settle Gate (prevents fork jams):**

Pressing Square (or auto-shoot firing) only actually flicks the fork if the spindexer has finished physically arriving at its commanded slot — not just whenever `flickState` happens to be idle. This stops a fast double-tap (or driver spam) from dropping the fork onto a ball mid-spin.

```
private void triggerFlick() {
    if (flickState == FlickState.IDLE && transferDown && spindexerSettled) {
        // ... fires
    }
}
```

A small rumble blip fires the moment `spindexerSettled` flips back to `true` — that's your "clear to fire" signal. Tune the wait with `SPINDEXER_SETTLE_SEC` (default 0.25s) if your servo needs more or less time.

---

### Intake State Machine

**Triangle Button Behavior:**

1. **If intake is OFF:** Start intake, move to first free slot.
2. **If intake is already ON:** Re-sync to the next free slot (misread recovery) — no full rescan, the dwell loop re-reads the new slot automatically.
3. **Auto-transition:** When spindexer fills, automatically switches to SHOOT mode, no driver input needed.

```
IDLE
  -> (Triangle, intake OFF)
RUNNING + DWELL (INTAKE_DWELL_SEC per slot, default 0.3s)
  -> (Color sensor stamps slot)
  -> (All 3 full?)
YES -> enterShootMode()
NO  -> Move to next free slot
  -> (Triangle again, intake already ON)
Re-sync to next free slot (no full rescan needed)
```

A separate full-rescan state machine (`tickRescan()`) still exists in code for backward compatibility, but Triangle no longer triggers it.

---

### Transfer Flick State Machine

Handles the fork mechanism (up/down) with timing and position recovery.

```
IDLE
  -> (triggerFlick(), gated on spindexerSettled)
WAIT_UP (FLICK_UP_SEC, default 0.9s)
  -> (Time elapsed)
WAIT_DOWN (FLICK_DOWN_SEC, default 1.5s)
  -> (Time elapsed)
  [Mark slot EMPTY, toggle position type for settling, reset flywheel to SPINNING]
IDLE
```

**Key Detail:** After the flick, the code toggles `PositionType` (SHOOT -> INTAKE -> SHOOT) so the spindexer servo re-centers correctly, then resets `flywheelState` back to 1 so the next ball is re-gated through the full settle/spin-up check rather than assuming the wheel is still at speed.

```
int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
```

---

### Auto-Shoot State Machine

Sequences multiple shots based on pattern detection. Uses the distance sensor to confirm ball exit.

```
IDLE
  -> (Cross pressed, flywheelState > 0, both flick/shoot states idle)
WAIT_FLYWHEEL (waits for flywheelState == 2 before firing)
  -> (flywheel ready + spindexer settled) -> fires, goes to CONFIRM
CONFIRM (waiting for distance sensor to clear)
  -> (Ball detected as clear?)
YES -> NEXT_BALL
NO  -> (Timeout after SHOT_TIMEOUT_SEC, default 1.5s) -> REFIRE_WAIT

REFIRE_WAIT
  -> (REFIRE_DELAY_SEC, default 0.5s, flick idle, forks down)
  -> back to WAIT_FLYWHEEL (re-confirms speed before refiring)

NEXT_BALL
  -> (Spindexer empty?)
YES -> IDLE (all balls shot)
NO  -> back to WAIT_FLYWHEEL (re-confirms speed before next shot)
```

**Why `WAIT_FLYWHEEL` exists:** every transition back into firing — first shot, refire, or next ball — re-checks that the flywheel is actually back at speed (`flywheelState == 2`) before triggering the fork again. This prevents a ball firing before the wheel has recovered from the previous shot.

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
If the pattern isn't locked, it just fires whichever slot has a ball, first match wins.

---

## 🎯 Turret Aiming System

### Odometry-Based Aiming (`aimAtGoal`)

When in **SHOOT mode** (and turret lock is off), the turret automatically tracks the goal using Pinpoint odometry:

```
angleToGoal = atan2(goalY - robotY, goalX - robotX)   // Field frame, degrees
turretAngle = angleToGoal - robotHeading + 270        // Robot-relative + mount offset
turretAngle = clamp(normalize360(turretAngle), 180, 360)
setToAngle(turretAngle)
```

The `+270` offset and `[180, 360]` clamp account for this turret's specific physical mounting range — don't copy these numbers onto a different turret without re-deriving them for its mount.

**Goal Coordinates (0-144 field system):**
- Blue: (0, 144)
- Red: (144, 144)

### Limelight Relocalization (MegaTag1)

Limelight uses **MegaTag1** (`getBotpose()`), not MegaTag2. MT1 solves both position and rotation fresh from the tag every frame, so it does **not** need robot orientation fed in beforehand — unlike MT2, which would otherwise need `setRobotOrientation()` called every loop before reading pose.

Two separate paths use it:

**1. Manual — DPad Down:**
```
double[] visionPose = Limelight.INSTANCE.getAveragedSnapshotPose(5);
if (visionPose != null && inBounds(visionPose)) {
    Pinpoint.INSTANCE.relocalizePositionFromTag(visionPose[0], visionPose[1]);
    gamepad1.rumble(1.0, 1.0, 150);   // success
} else {
    gamepad1.rumble(0.5, 0.0, 400);   // no tag / failed
}
```
Averages 5 samples, corrects X/Y only — **heading is left completely untouched** by design (`relocalizePositionFromTag`'s own contract).

**2. Passive — while stationary in INTAKE mode:**
```
if (mode == RobotMode.INTAKE && relocTimer.seconds() > 0.5 && isStill) {
    double[] llPose = Limelight.INSTANCE.getSnapshotPose();
    if (llPose != null && inBounds(llPose)) {
        double error = Math.hypot(llX - ppX, llY - ppY);
        if (error < 20) {
            // 95% odometry / 5% vision blend - gentle correction, not a snap
            Pinpoint.INSTANCE.relocalizePositionFromTag(blendedX, blendedY);
        }
    }
}
```
This only runs in INTAKE mode while the sticks are near-zero, and only blends in vision if the disagreement with current odometry is under 20 inches (guards against a single bad vision frame yanking the position).

**Note:** there is currently no heading-only relocalization path in code (`relocalizeFull()` exists in `Pinpoint.` for X/Y + heading together, but nothing in `Teleop.` currently calls it).

### Turret Lock (Right Trigger)

Press **Right Trigger** to toggle turret lock. When locked, the turret points at `TURRET_PARKED_ANGLE` (**270°**) instead of tracking the goal. Useful if vision/odometry is unreliable.

```
if (!turretLock) {
    Turret.INSTANCE.aimAtGoal(alliance, goalId);
} else {
    Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
}
```

---

## 📊 Lookup Tables (Shooter Tables)

The turret uses **bilinear interpolation** to automatically select:
- **Flywheel RPM** based on (X, Y) position from goal
- **Hood angle** (servo position 0.0-1.0) based on (X, Y) position from goal

These are pre-tuned points loaded from `ShooterTablesV2`:

```
targetVelo = Turret.INSTANCE.distanceToVelocity(px, py, alliance);
targetHood = Turret.INSTANCE.distanceToPosition(px, py, alliance);
```

**Manual Overrides (FTC Dashboard):**
- `HOOD_OVERRIDE`: Set to 0.0-1.0 to manually set hood. Default -1 (use table).
- `VELO_OVERRIDE`: Set to >0 to manually cap flywheel RPM. Default -1 (use table).

---

## ⚙️ Tunable Constants

| Constant | Default | Purpose |
|----------|---------|---------|
| `TURRET_PARKED_ANGLE`* | 270° | Turret angle when locked or in intake mode. *(defined in `Turret.`, not `Teleop.`)* |
| `INTAKE_DWELL_SEC` | 0.3s | Time to wait before reading color sensor. |
| `FLICK_UP_SEC` | 0.9s | Time transfer forks stay raised. |
| `FLICK_DOWN_SEC` | 1.5s | Time to wait after forks return before next action. |
| `SHOT_TIMEOUT_SEC` | 1.5s | Max wait for distance sensor before refire. |
| `REFIRE_DELAY_SEC` | 0.5s | Pause between failed shots. |
| `RESCAN_MOVE_SEC` | 0.15s | Time to settle servo before reading color (used by the legacy rescan path). |
| `SPINDEXER_SETTLE_SEC` | 0.25s | Min time after commanding a new SHOOT-mode slot before the fork is allowed to fire. |
| `HOOD_OVERRIDE` | -1 | Manual hood override, -1 = use lookup table. |
| `VELO_OVERRIDE` | -1 | Manual flywheel RPM cap, -1 = use lookup table. |

---

## 🔄 Mode Switching

The robot has two modes:

### **INTAKE Mode**
- Intake motor spinning (if `intakeRunning`)
- Spindexer rotating to free slots
- Turret parked at `TURRET_PARKED_ANGLE`, flywheel off
- Passive Limelight relocalization active while stationary
- **Exit:** Auto-switches to SHOOT when all 3 slots full, or via DPad Right / Triangle-from-shoot

### **SHOOT Mode**
- Intake idle
- Spindexer rotating to filled slots in pattern order
- Turret tracking goal (odometry-based `aimAtGoal`, unless turret-locked)
- Flywheel spinning up, gated firing via the spindexer-settle check
- **Exit:** Manual (Triangle, Circle, or DPad Left) or automatically when spindexer empties

**Manual Mode Force:**
- **DPad Left:** Force INTAKE mode (and starts intake running)
- **DPad Right:** Force SHOOT mode

---

## ⚡ Initialization Flow

```
init()
  - Init all subsystems (Drivetrain, DistanceSensor, Intake, Spindexer, Transfer, Turret, Pinpoint, Limelight)
  - Transfer forks DOWN (aggressive)
  - MatchPattern reset

init_loop()
  - Wait for alliance selection (DPad Up = Blue, Down = Red)
  - Detect field pattern (obelisk motif) via MatchPattern.tryDetect()
  - Hold Cross to calibrate turret to parked position
  - Seed starting pose from MT1 if a tag is visible, else fall back to alliance corner

start()
  - Reset MatchPattern, park turret
  - Pre-match scan: rotate through all 3 slots, read + stamp colors (400ms settle each)
  - If full -> enterShootMode(); else -> enterIntakeMode()

loop() [repeated every ~loop-time]
  - Update Pinpoint odometry, try pattern detection if not locked
  - Read inputs + process state machines (rescan, flick, auto-shoot)
  - Drive robot
  - Run intake dwell/color-read logic (if in INTAKE mode)
  - Auto-aim turret + run shooter tables (if in SHOOT mode)
  - Track spindexer settle state, rumble when clear to fire
  - Passive Limelight relocalization (if stationary in INTAKE mode)
  - Update telemetry (FTC Dashboard + Driver Station)
```

---

## 🐛 Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Robot won't switch to SHOOT | Spindexer not detected as full | Manual: Press DPad Right |
| Color sensor keeps misreading | Lighting/sensor angles | Press Triangle to re-sync to next free slot. Adjust HSV ranges in `Spindexer`. |
| Turret oscillates wildly | PD tuning or bad odometry | Increase `turretKd` or enable turret lock (Right Trigger) |
| Odometry drifts | Encoder slip | DPad Down to relocalize position from Limelight, or DPad Up to hard-reset at the corner |
| Square spam jams the fork on a ball | *(Fixed)* Previously no settle check existed between commanding a spindexer move and allowing a fire | Now gated by `spindexerSettled` / `SPINDEXER_SETTLE_SEC` — tune the constant up if jams still occur on your servo |
| Auto-shoot fires wrong ball | Pattern not locked | Check obelisk detection in `init_loop()`. Manually correct with bumpers. |

---

## 🎓 Advanced Tips

1. **Pre-match Scan:** The robot scans all 3 slots at start. Pre-load balls in the **same order** you want to shoot to avoid confusion.

2. **Misread Recovery:** If a slot gets misread mid-match, press Triangle while intake is running to re-sync to the next free slot — no full rescan needed.

3. **Odometry Drift:** DPad Down passively corrects X/Y position from Limelight without touching heading. Use DPad Up at a known corner for a full hard reset (position + heading) if drift gets bad.

4. **Manual Override:** Use `HOOD_OVERRIDE` and `VELO_OVERRIDE` on FTC Dashboard during testing to dial in shot distance without redeploying.

5. **Turret Lock:** If odometry is unreliable in a specific area, press Right Trigger to lock turret, then manually aim and fire using hood/velocity overrides.

---

## 📞 Support

For detailed subsystem info, see `Subsystems_README.md`.  
For tuning info, see `Tuning_README.md`.