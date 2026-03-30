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

## What Changed — March 30, 2026 Update
## Turret Aiming — How It Works:

The turret uses a two-stage aiming system to lock onto the goal.

**Stage 1 — Coarse aim (Pinpoint odometry)**
As soon as shoot mode is active, `followGoalOdometryPositional()` runs every loop tick. It takes the robot's current XY position and heading from Pinpoint, computes the angle to the goal, and sets the turret there. This gets the turret close fast but isn't precise enough on its own — odometry drift and mounting offsets mean it can be a few degrees off.

**Stage 2 — Fine tune (Limelight tx)**
Once the turret angle error drops below `fineTuneThresholdDeg` (default 5°), `fineTuneActive` flips to true and the Limelight turns on. From here, `getTx()` reads the horizontal offset in degrees between the camera crosshair and the goal AprilTag. If the tag is visible and the offset is larger than `fineTuneBangBangDeg` (default 0.5°), it nudges `angleOffset` by `fineTuneNudge` in the correct direction each tick. This offset carries forward into `followGoalOdometryPositional()` so the two stages stay in sync. If the tag disappears mid-match, it falls back to pure odometry automatically.

**Why not just use Limelight the whole time?**
The Limelight's field of view is narrow. Relying on it from the start means if the turret starts far off angle the tag won't even be in frame. Odometry gets it close enough first, then Limelight takes over for the final correction.

**Turret lock override**
Right Trigger toggles `turretLock`, which bypasses both stages entirely and holds the turret at the fixed `turretAngle` park position. Useful if tracking is fighting the driver or the tag is being blocked.

## New Controls:
**DPad Left / DPad Right** now force mode switches regardless of current state.
- **DPad Left** — force intake mode: kills flywheel, turns intake on, resets spindexer to first free slot
- **DPad Right** — force shoot mode: stops intake, spins flywheel up to state 1, moves spindexer to next ball to shoot
Both reset `autoShootState` and `flickState` to IDLE so you're never left mid-sequence after a forced switch.
Previously both DPad Left and Right just reset `hoodOverride` to -1. That behavior is gone.
- **DPad Up** — reset odometry + zero turret offset
- **DPad Down** — zero turret angle offset
- **Left Trigger** — reverse intake (if intake is on)
- **Right Trigger** — toggle turret lock
- **More Code cleanups re added auto stamping on pickup from *outreach.java*,**

## Turret.java
- No functional changes
- Removed Bilinear Lookup Tables from **Turet.java**, and made a new Ultil Opmode Called **"ShooterTables.java"**

## Spindexer.java
- No changes — slot stamping, dwell logic, and color detection unchanged

## DistanceSensor.java

Wraps the REV 2m distance sensor mounted at the shooter exit to detect whether a ball is sitting in the shooter or has already left.

**Hardware name:** `distanceSensor`

**How it works:** reads distance in cm every time it's called. If the reading falls between `BALL_MIN_CM` and `BALL_MAX_CM`, a ball is considered present. Anything outside that range (too close or too far) is considered clear.

- `isBallPresent()` — true when a ball is detected in the shooter
- `isClear()` — inverse; used by the auto-shoot state machine to confirm a ball has left before advancing to the next one
- `getDistanceCM()` — raw distance reading; returns 999 if the sensor failed to initialize

`BALL_MIN_CM` (default 3.0) and `BALL_MAX_CM` (default 15.0) are tunable live on FTC Dashboard.

## Controls and Drivestation Confgiuration Links

[Controls Reference Link](https://docs.google.com/document/d/1A3iNGGY-CFJ41gOVWxeUdjWmHU4-4ooo/edit?usp=sharing&ouid=105244545883819724199&rtpof=true&sd=true)

[DriveStation Configs](https://docs.google.com/document/d/1f5MVQA72TZ1hu_ieV58IywkEtbo__3kph1g5qZGos4g/edit?usp=sharing)

# Removed ALL Autos..
**Why?** 
**I noticed that the old autos from Cheicks code was flawed and dated and I want to create my owns instead of stealing his code**
- I will more than likley create autos once all teleops are finished and the bot works really well

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
- Servo positions: (needs to be updated)
- Color detection via HSV thresholds, tunable on FTC Dashboard
- Both slot-stamping and empty/full flag bugs fixed 

#### `Turret.java`
- Flywheel shooter (2× DcMotorEx), turret rotation (DcMotor), hood servo, analog encoder
- Bang-bang velocity control with deadband — holds current power within threshold to prevent oscillation
- Turret PD position control: turretKp=0.01, turretKd=0.001, capped at maxPower=0.45
- Two-stage aiming: coarse odometry aim via Pinpoint → Limelight tx fine-tune once within fineTuneThresholdDeg
- Turret physical limits clamped to 180°–360° (center=270°, ±90° travel)
- Shooter motors set to FLOAT zero power behavior
- Velocity and hood position looked up from bilinear interpolation tables (ShooterTables) based on robot XY position
- turretOffSet=250 maps raw analog encoder voltage to degrees
- Hardware names: shoot1, shoot2, turret, hood, turretEncoder

#### `Limelight.java` (`org.firstinspires.ftc.teamcode.Vision`)
- INSTANCE singleton
- Wrapped in try-catch on init — robot won't crash if Limelight is unplugged
- Null guards on all methods
- getTx(tagId), distanceFromTag(tagId), patternFromObelisk() — all return safe defaults on null/missing
- Tag IDs: BLUEGOAL=20, GPP=21, PGP=22, PPG=23, REDGOAL=24

#### `distanceSensor.java`
- Wraps REV 2m distance sensor
- isBallPresent() / isClear() — checks distance against tunable BALL_MIN_CM / BALL_MAX_CM window
- Returns 999cm on null sensor (safe default, never falsely triggers ball-present)

#### `MatchPattern.java` (`org.firstinspires.ftc.teamcode.Utils`)
- Stores and locks the detected field motif pattern (GPP / PGP / PPG)
- `tryDetect()` called every `init_loop()` tick until pattern is confirmed
- `isLocked()` prevents re-scanning mid-match
- `reset()` called on init to clear between matches

#### `Interpolator.java`
- 2D lookup table with bilinear interpolation
- Falls back to nearest-neighbor if query point is outside the grid or grid is missing corners
- Used by Turret to look up shooter velocity and hood position from robot XY

#### `ShooterTables.java` (New)
- Static loader methods that populate Interpolator instances for both alliances
- loadBlueShooter, loadBlueHood, loadRedShooter, loadRedHood
- Points sourced from field testing — mirror red/blue across the 144" centerline

---

## OpModes

### Teleop (`org.firstinspires.ftc.teamcode.teleop.Teleop`)
Full competition driver-controlled period. Alliance hardcoded Blue (Red support present). Pattern detected during init, locked at match start. Full color-matched shoot sequencing, auto-spindexing with dwell, jam retry, Limelight + Pinpoint turret aim.

### Tuning OpModes (`org.firstinspires.ftc.teamcode.tuning`)

- **`FullTest.java`** — Complete mechanism test with Limelight. Standardized controls, dwell logic, jam retry. Alliance hardcoded Blue.
- **`DataCollection.java`** Full BiLinear Lookup Table testing. 
- **`Outreach.java`** — Demo/event OpMode. Simplified for non-drivers. Origin of the dwell + auto-mode-switching logic now used everywhere.
- **`TestTurret.java`** — Isolated turret and encoder tuning
- **`TestShooter.java`** — Flywheel velocity tuning
- **`TestSpindexer.java`** — Spindexer position and color sensor tuning
- **`TestLimelight.java`** — AprilTag detection and distance testing
- **`TestHood.java`** — Hood servo position tuning
- **`TestDriveTrain.java`** - Simple Mechnuam DriveTrain test to make sure wheels are working

### Autonomous (`org.firstinspires.ftc.teamcode.Auto`)

- **Removed until bot works flawlessly in teleop**


---

## TODO — Next update or so..

- [ ] Make First 9 Ball Auto
- [ ] Re-tune bilinear interpolator tables from new robot position data
- [ ] Verify starting position coordinates match actual field setup

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
- **`Aliance.java`** - Simple aliance picker enum
- **`ShooterTables.java`** - Static table picker for Bilinear Lookup Table
- **'SensorColor.java'** - Currently unused may be used for better HSV color picking for spindexer.java
---

## Future Goals

- **Shooting on the move** (Luke's request) — turret already auto-aims every tick, the turret already auto-aims every tick, the remaining work is lead-target compensation: predicting where the goal will be relative to the robot by the time the ball arrives, then offsetting the aim angle and velocity accordingly. Ethan from 10195 got a slight version of this on their robot — the math is based on robot velocity from Pinpoint and known ball flight time.
- Full 12-ball autonomous (maybe even 26 ball)
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
├── Utils/              # Aliance, MatchPattern, Interpolator, SensorColor, ShooterTables
└── Configurations/     # ConfigureColorRangefinder
```

---

## Offseason Development

This refactor was worked on by:
- **Kristian** — lead programmer, full codebase refactor, spindexer overhaul, color matching, control standardization, dwell logic
- **Luke** — direction, hardware decisions, feature requests

- **Built on top of Cheick's** original logic and field data. The goal was never to throw away what he built — it was to make it work reliably and be understandable to anyone on the team going forward.