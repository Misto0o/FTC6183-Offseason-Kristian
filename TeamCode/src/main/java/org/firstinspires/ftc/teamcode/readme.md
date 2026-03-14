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

## What Changed — March 14, 2026 Update

### 🏆 Color Matching (Biggest Feature — Previously 6183's Hardest Problem)

The single biggest unsolved problem on 6183 was getting the robot to shoot balls in the correct order matching the field motif pattern. **This is now working.**

During `init_loop()`, the Limelight scans the obelisk AprilTag and detects the pattern (GPP / PGP / PPG). The pattern is stored once and locked — since it resets every match, we only need to scan once and the robot remembers the correct shoot order for the entire match. The spindexer now automatically pre-positions to the correct color slot based on the detected pattern before each shot, so balls are fired in the exact order the field expects.

This was implemented via `MatchPattern.java` which holds the detected pattern and locks it once confirmed. `getTargetColor(step)` and `findColorPosition(color)` in Teleop walk through the pattern sequence and route the spindexer to the right slot every time.

### 🔄 Spindexer Overhaul

- **Fixed slot-stamping bug** — `setColor()` previously always wrote to `currentPosition` instead of the passed-in position, causing wrong slots to be marked empty during shoot sequences
- **Fixed empty/full flag bug** — `checkSpindexerState()` now properly resets flags at the start of each check instead of letting them get stuck
- **~25% faster indexing** — servo position tuning and dwell timing improvements
- **Jam detection and auto-retry** — after each flick, the robot waits `shootVerifySec` then reads the color sensor at that slot. If the ball is still there (jammed), it automatically re-stamps and re-fires. No driver input needed. `shootVerifySec` is tunable on FTC Dashboard (recommended: 0.5–0.6s)

### 🎮 Standardized Controls Across All OpModes

Controls were inconsistent and confusing across Teleop, FullTest, and DataCollection. Full Test and Teleop OpModes now use the same button layout so drivers have identical muscle memory everywhere:

| Button | Action |
|---|---|
| Triangle | Intake mode on |
| Circle | Intake off / switch to shoot mode |
| Cross | Manual transfer flick |
| Square ×1 | Spin up flywheel |
| Square ×2 | Shoot (after ready rumble) |
| Square (during spin) | Emergency kill flywheel |
| Left Bumper | Spindexer next position |
| Right Bumper | Spindexer previous position |
| DPad Left | Reverse intake (hold) |
| DPad Down | Zero turret angle offset |
| Right Trigger | Nudge aim right (−0.1°) |
| Left Trigger | Nudge aim left (+0.1°) |

### 🎪 New: Outreach OpMode

Added `Outreach.java` — a dedicated OpMode for demos and events. It runs the full robot but is simplified so anyone can drive it without competition context.

Key feature: **full auto-spindexing with dwell logic** built in. The outreach OpMode was actually where the dwell-based auto-indexing logic was first fully figured out — reading color, waiting a confirmed dwell period before stamping the slot, then automatically rotating to the next free position and switching between intake/shoot mode based purely on spindexer sensor state. Once proven here, that same logic was ported into Teleop, FullTest, and DataCollection.

### 🔧 Dwell Logic (Now In All OpModes)

Previously only Teleop had dwell logic — FullTest and DataCollection called periodic() raw every loop tick which caused unreliable color reads. The fix was adding a short confirmed wait before stamping a color into a slot — the sensor reads the color, waits intakeDwellSec (default 0.3s, tunable on dashboard) to confirm it's a real read and not a flicker, then stamps the slot and advances the spindexer. If the spindexer fills up completely it automatically switches to shoot mode, and if there's still free slots it rotates to the next one. All three OpModes now share this exact same behavior.

`intakeDwellSec` is tunable on FTC Dashboard (default: 0.3s).

---

## Subsystems (`org.firstinspires.ftc.teamcode.robot`)

#### `Drivetrain.java`
- Plain mecanum drive, `getInstance()` singleton
- `drive(y, x, rx)` — raw mecanum math
- Brake mode on all motors, fl/fr reversed to match hardware

#### `Intake.java`
- `on()`, `idle()`, `reverse()` — plain void methods
- Fixed core bug: Cheick's version returned a `SetPower` Command that was never executed

#### `Pinpoint.java`
- GoBilda Pinpoint odometry computer
- Offsets: 4.5" strafe, −7.125" forward
- goBILDA 4-bar pod, FORWARD/REVERSED encoder directions
- `pinpoint.resetPosAndIMU()` on init to clear stale position between runs

#### `Transfer.java`
- Two servos (`leftFork`, `rightFork`) for ball transfer mechanism
- `transferUp()` / `transferDown()` — plain void
- Servo positions: leftUp=1, leftDown=0, rightUp=0, rightDown=1

#### `Spindexer.java`
- Three-position ball indexer with two NormalizedColorSensors
- Hardware: `spinServo`, `leftColorSensor`, `rightColorSensor`
- Servo positions: intake={0.05, 0.42, 0.79}, shoot={0.25, 0.62, 1.0}
- Color detection via HSV thresholds, tunable on FTC Dashboard
- Both slot-stamping and empty/full flag bugs fixed (see above)

#### `Turret.java`
- Flywheel shooter (2× DcMotorEx), turret rotation (DcMotor), hood servo, analog encoder
- Bang-bang velocity control with deadband — prevents oscillation
- Turret PD control: `turretKp=0.01`, `turretKd=0.001`, `maxPower=0.45`
- Fixed `distanceToVelocity()` dead zone that returned 0 for mid-field positions
- Fixed `isAtVelocity()` false-ready when target is 0
- Shooter motors set to `FLOAT` zero power behavior
- Turret physical limits clamped to 180°–360° (center=270°, ±90° travel)
- Full bilinear interpolator tables for blue/red velocity and hood position (26 points each)

#### `Limelight.java` (`org.firstinspires.ftc.teamcode.Vision`)
- Wrapped in try-catch — robot won't crash if Limelight is unplugged
- Null guards on all methods
- Tag IDs: BLUE_GOAL=20, RED_GOAL=24, GPP=21, PGP=22, PPG=23

#### `MatchPattern.java` (`org.firstinspires.ftc.teamcode.Utils`)
- Stores and locks the detected field motif pattern (GPP / PGP / PPG)
- `tryDetect()` called every `init_loop()` tick until pattern is confirmed
- `isLocked()` prevents re-scanning mid-match
- `reset()` called on init to clear between matches

---

## OpModes

### Teleop (`org.firstinspires.ftc.teamcode.teleop.Teleop`)
Full competition driver-controlled period. Alliance hardcoded Blue (Red support present). Pattern detected during init, locked at match start. Full color-matched shoot sequencing, auto-spindexing with dwell, jam retry, Limelight + Pinpoint turret aim.

### Tuning OpModes (`org.firstinspires.ftc.teamcode.tuning`)

- **`FullTest.java`** — Complete mechanism test with Limelight. Standardized controls, dwell logic, jam retry. Alliance hardcoded Blue.
- **`DataCollection.java`** — Full robot test with verbose odometry telemetry. Turret velocity/hood auto-set from bilinear table. Square resets odometry. Standardized controls, dwell logic, jam retry.
- **`Outreach.java`** — Demo/event OpMode. Simplified for non-drivers. Origin of the dwell + auto-mode-switching logic now used everywhere.
- **`TestTurret.java`** — Isolated turret and encoder tuning
- **`TestShooter.java`** — Flywheel velocity tuning
- **`TestSpindexer.java`** — Spindexer position and color sensor tuning
- **`TestLimelight.java`** — AprilTag detection and distance testing
- **`TestHood.java`** — Hood servo position tuning

### Autonomous (`org.firstinspires.ftc.teamcode.Auto.SoloAuto`)
Rewritten from NextFTC command chains to `LinearOpMode` with Pedro Pathing.

- **`BlueCloseSixBallAuto.java`** ✅ — Drives to shoot position, shoots 3, intakes row 1, shoots 3, intakes row 2, shoots 3
- `BlueCloseNineBallAuto`, `RedCloseSixBallAuto`, `BlueCloseTwelveBallAuto` — pending

---

## TODO — Next update or so..

- [ ] Finish rewriting remaining autonomous OpModes
- [ ] Re-tune turret — `threshold`, `turretKp`, `turretKd` on FTC Dashboard after hardware changes
- [ ] Re-tune bilinear interpolator tables from new robot position data
- [ ] Verify hardware config names: `leftColor`, `rightColor`, `spinServo`, `hood`, `turretEncoder`, `shoot1`, `shoot2`, `turret`
- [ ] Confirm turret physical stop angles match 180°–360° clamp
- [ ] Verify starting position coordinates match actual field setup
- [ ] Rewire robot so the intake isnt eating the turret

---

## Pedro Pathing (`org.firstinspires.ftc.teamcode.Pedro`)

No changes made. Used for autonomous path following only.

- Mass: 35kg
- Forward zero power acceleration: −43.18
- Lateral zero power acceleration: −64
- Translational PIDF: (0.175, 0, 0.01, 0.03)
- Heading PIDF: (1, 0, 0.02, 0.03)
- Drive PIDF: (0.0025, 0, 0.00001, 0.6, 0.01)

---

## Utilities (`org.firstinspires.ftc.teamcode.Utils`)

- **`Aliance.java`** — Simple enum: `BLUE`, `RED`
- **`MatchPattern.java`** — Pattern detection, locking, and reset logic
- **`Interpolator.java`** — 2D bilinear interpolation with nearest-neighbor fallback
- **`SensorColor.java`** — HSV calibration OpMode for color sensor tuning, not used in match code

---

## Future Goals

- **Shooting on the move** (Luke's request) — turret already auto-aims every tick, the turret already auto-aims every tick, the remaining work is lead-target compensation: predicting where the goal will be relative to the robot by the time the ball arrives, then offsetting the aim angle and velocity accordingly. Ethan from 10195 got a slight version of this on there robot — the math is based on robot velocity from Pinpoint and known ball flight time.
- Full 12-ball autonomous
- Red alliance autonomous

---

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Outreach/           # Outreach Opmode for events
├── robot/              # All subsystems (Drivetrain, Intake, Turret, Spindexer, Transfer, Pinpoint)
├── teleop/             # Teleop.java
├── tuning/             # All tuning and test OpModes
├── Auto/               # Autos like solo auto paired ect
├── Pedro/              # Pedro Pathing constants and tuning
├── Vision/             # Limelight.java, DistanceSensor
├── Utils/              # Aliance, MatchPattern, Interpolator, SensorColor
└── Configurations/     # ConfigureColorRangefinder
```

---

## Offseason Development

This refactor was worked on by:
- **Kristian** — lead programmer, full codebase refactor, spindexer overhaul, color matching, control standardization, dwell logic
- **Luke** — direction, hardware decisions, feature requests

- **Built on top of Cheick's** original logic and field data. The goal was never to throw away what he built — it was to make it work reliably and be understandable to anyone on the team going forward.