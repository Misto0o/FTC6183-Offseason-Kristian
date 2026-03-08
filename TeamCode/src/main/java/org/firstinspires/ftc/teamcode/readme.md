# FTC 6183 Loki — Robot Recode 🤖🔥

This repository contains the fully refactored robot software for FTC Team 6183 Loki.
Over the course of this offseason, the entire codebase was rebuilt from the ground up — removing a third-party command framework, fixing bugs, and rewriting every subsystem in clean, plain FTC SDK code.

---

## Why We Rewrote Everything

Our previous lead programmer (Cheick) built the original codebase on top of **NextFTC**, a third-party command-based framework. While the logic underneath was solid, the framework was causing real problems:
- Subsystem methods returned `Command` objects that **silently did nothing** if you forgot to call `.run()` or `.schedule()` on them — motors never moved and there was no error
- `Robot.java` used blocking `sleep()` calls inside command sequences that froze the entire loop
- The code was extremely difficult for new programmers to read or debug
- Removing the framework required touching every single file

The decision was made to strip NextFTC out entirely and rewrite everything in **plain iterative OpMode** style — the standard FTC SDK that every FTC programmer already knows.

---

## What Changed

### Framework Removal
- Removed all `NextFTCOpMode`, `SubsystemGroup`, `Command`, `SequentialGroupFixed`, `Delay`, `InstantCommand`, `WaitUntil`, `RunToVelocity`, `RunToPosition`, `SetPosition`, `SetPower`, and all other NextFTC classes
- Removed `Robot.java` entirely — Teleop now talks directly to subsystems
- All subsystem methods changed from returning `Command` objects to plain `void`
- All delays replaced with `ElapsedTime`-based state machines
- All button handling uses `prevButton` boolean edge detection pattern

### Subsystems Rewritten (`org.firstinspires.ftc.teamcode.robot`)

#### `Drivetrain.java`
- Plain mecanum drive, `getInstance()` singleton
- `drive(y, x, rx)` — raw mecanum math, same as original
- Brake mode on all motors, fl/fr reversed to match hardware
- Removed unused IMU (field-centric not used in teleop)

#### `Intake.java`
- `on()`, `idle()`, `reverse()` — plain void methods
- Fixed the core bug: Cheick's version returned a `SetPower` Command that was never executed, so the motor never actually moved

#### `Pinpoint.java`
- GoBilda Pinpoint odometry computer
- Offsets: 4.5" strafe, -7.125" forward
- goBILDA 4-bar pod, FORWARD/REVERSED encoder directions
- Added `pinpoint.resetPosAndIMU()` on init to clear stale position between runs

#### `Transfer.java`
- Two servos (`leftFork`, `rightFork`) for ball transfer mechanism
- `transferUp()` / `transferDown()` — plain void
- Servo positions: leftUp=1, leftDown=0, rightUp=0, rightDown=1

#### `Spindexer.java`
- Three-position ball indexer with two NormalizedColorSensors
- Hardware: `spinServo`, `leftColor`, `rightColor`
- Servo positions: intake={0.05, 0.42, 0.79}, shoot={0.25, 0.62, 1.0}
- Color detection via HSV thresholds, tunable on FTC Dashboard
- **Bug fixed:** Cheick's `setColor()` always wrote to `currentPosition` instead of the passed-in position, causing wrong slots to be marked empty during shoot sequences
- **Bug fixed:** `checkSpindexerState()` now properly resets `empty`/`full` flags at the start of each check instead of letting them get stuck

#### `Turret.java`
- Flywheel shooter (2x DcMotorEx), turret rotation (DcMotor), hood servo, analog encoder
- **Bang-bang velocity control with deadband** — holds current power within `threshold` to prevent oscillation. Cheick's original flip-flopped every tick
- **Turret PD control** — `turretKp=0.01`, `turretKd=0.001`, `maxPower=0.45`
- **Bug fixed:** `distanceToVelocity()` had a `y>60||y<40` dead zone condition that returned 0 for any robot position with y between 40-60 (mid-field). Removed entirely
- **Bug fixed:** `isAtVelocity()` now returns `false` when target velocity is 0, preventing false "ready" signals
- **Improvement:** Shooter motors set to `FLOAT` zero power behavior so flywheels spin down naturally instead of braking
- Turret physical limits clamped to 180°–360° (center=270°, ±90° travel)
- Full bilinear interpolator tables for blue/red shooter velocity and hood position (26 points each)
- `followGoalOdometryPositional(Aliance)` replaces the old NextFTC Command version — call every loop tick

#### `Limelight.java` (`org.firstinspires.ftc.teamcode.Vision`)
- Wrapped in try-catch so robot doesn't crash if Limelight is unplugged
- Null guards on all methods
- `distanceFromTag()` uses plain `Math.sqrt()` — removed Pedro Vector import
- Removed dead `relocalizeFromCamera()` method
- Tag IDs: BLUE_GOAL=20, RED_GOAL=24, GPP=21, PGP=22, PPG=23

---

### Teleop (`org.firstinspires.ftc.teamcode.teleop.Teleop`)

Full driver-controlled period. Alliance selected during `init_loop()`.

| Button | Action |
|---|---|
| Circle | Toggle intake on/off |
| Cross | Manual transfer flick |
| Triangle | Toggle intake/shoot mode |
| Square ×1 | Spin up flywheel |
| Square ×2 | Shoot (after ready rumble only) |
| Square (during spin) | Emergency kill flywheel |
| Left Bumper | Spindexer next position |
| Right Bumper | Spindexer previous position |
| DPad Up | Toggle turret lock (parks at 270°) |
| DPad Down | Zero turret angle offset |
| DPad Left | Reverse intake |
| Right Trigger | Nudge aim right (-0.1°) |
| Left Trigger | Nudge aim left (+0.1°) |

Key behaviors:
- Flywheel 3-state machine: off → spinning up → ready (rumbleBlips when at speed)
- Turret parks at 270° in intake mode, auto-aims via odometry in shoot mode
- Velocity and hood position dynamically pulled from interpolator table based on robot position
- Transfer flick is a non-blocking state machine using `ElapsedTime`
- Ball detection triggers gamepad rumble (left=green, right=purple)

---

### Tuning OpModes (`org.firstinspires.ftc.teamcode.tuning`)

All tuning OpModes stripped of NextFTC and fixed:

- **`DataCollection.java`** — Full robot test with odometry, turret auto-aim, spindexer, and color detection. Fixed `Intake.on().run()` → `Intake.on()`
- **`FullTest.java`** — Complete mechanism test with Limelight. Replaced `Turret.on()`/`Turret.off()` with `setVelocity()`
- **`TestTurret.java`** — Isolated turret and encoder tuning
- **`TestShooter.java`** — Flywheel velocity tuning
- **`TestSpindexer.java`** — Spindexer position and color sensor tuning
- **`TestLimelight.java`** — AprilTag detection and distance testing
- **`TestHood.java`** — Hood servo position tuning

---

### Autonomous (`org.firstinspires.ftc.teamcode.Auto.SoloAuto`)

Cheick's autos used `NextFTCOpMode` + `SequentialGroupFixed` + `Delay` command chains. These have been rewritten as `LinearOpMode` with Pedro Pathing.

- **`BlueCloseSixBallAuto.java`** ✅ — Rewritten. Drives to shoot position, shoots 3, intakes row 1, shoots 3, intakes row 2, shoots 3.
- `BlueCloseNineBallAuto`, `RedCloseSixBallAuto`, `BlueCloseTwelveBallAuto` — pending (same structure, different paths)

Key changes in auto:
- `NextFTCOpMode` → `LinearOpMode`
- `FollowPath` Command → `follower.followPath()` with busy-wait loop
- `Delay` → `ElapsedTime` timer in `tickPeriodicFor()`
- `waitToShoot()` Command → `waitForVelocity()` polling `isAtVelocity()` with 3s timeout
- All subsystems ticked every loop via `updatePeriodicSystems()`
- Flywheel stays warm at 500 ticks/sec during intake driving

---

### Pedro Pathing (`org.firstinspires.ftc.teamcode.Pedro`)

**No changes made.** Pedro Pathing is used for autonomous path following only and was already clean. Constants tuned for this robot:

- Mass: 35kg
- Forward zero power acceleration: -43.18
- Lateral zero power acceleration: -64
- Translational PIDF: (0.175, 0, 0.01, 0.03)
- Heading PIDF: (1, 0, 0.02, 0.03)
- Drive PIDF: (0.0025, 0, 0.00001, 0.6, 0.01)

---

### Utilities (`org.firstinspires.ftc.teamcode.Utils`)

- **`Aliance.java`** — Simple enum: `BLUE`, `RED`
- **`Interpolator.java`** — 2D bilinear interpolation with nearest-neighbor fallback. Used for shooter velocity and hood position lookup tables
- **`SensorColor.java`** — HSV calibration OpMode for color sensor tuning, not used in match code

---

## Pending / Before First Match

- [ ] Rewire robot and zero turret encoder
- [ ] Verify hardware config names: `leftColor`, `rightColor`, `spinServo`, `hood`, `turretEncoder`, `shoot1`, `shoot2`, `turret`
- [ ] Tune `threshold` (deadband width), `turretKp`, `turretKd` on FTC Dashboard
- [ ] Confirm turret physical stop angles match 180°–360° clamp
- [ ] Verify starting position coordinates match actual field setup
- [ ] Finish rewriting remaining autonomous OpModes
- [ ] Tune interpolator tables from new robot position data

---

## Future Goals

- **Shooting on the move** (Luke's request) — turret already auto-aims every tick, the remaining work is lead-target angle correction and velocity compensation based on robot velocity from Pinpoint
- Full 12-ball autonomous
- Red alliance autonomous

---

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── robot/              # All subsystems (Drivetrain, Intake, Turret, Spindexer, Transfer, Pinpoint)
├── teleop/             # Teleop.java
├── tuning/             # All tuning and test OpModes
├── Auto/
│   └── SoloAuto/       # Autonomous OpModes
├── Pedro/              # Pedro Pathing constants and tuning
├── Vision/             # Limelight.java
├── Utils/              # Aliance, Interpolator, SensorColor
└── Configurations/     # ConfigureColorRangefinder
```

---

## Offseason Development

This refactor was worked on by:
- **Kristian** — lead programmer, full codebase refactor
- **Luke**  — direction, hardware decisions, feature requests

Built on top of Cheick's original logic and field data. The goal was never to throw away what he built — it was to make it work reliably and be understandable to anyone on the team going forward.