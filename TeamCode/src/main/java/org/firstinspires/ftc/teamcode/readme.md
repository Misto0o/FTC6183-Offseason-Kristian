# FTC 6183 Loki — Robot Recode 🤖🔥

This repository contains the fully refactored robot software for FTC Team 6183 Loki.
Over the course of this offseason, the entire codebase was rebuilt from the ground up — removing a third-party command framework, fixing bugs, and rewriting every subsystem in clean, plain FTC SDK code.

---

## TL;DR Welcome!
- Full robot codebase rewritten from NextFTC → FTC SDK
- New autos
- Bug fixes + code changes

---

## Why We Rewrote Everything

Our previous lead programmer (Cheick) built the original codebase on top of **NextFTC**, a third-party command-based framework. While the logic underneath was solid, the framework was causing real problems:
- Subsystem methods returned `Command` objects that **silently did nothing** if you forgot to call `.run()` or `.schedule()` on them — motors never moved and there was no error
- `Robot.java` used blocking `sleep()` calls inside command sequences that froze the entire loop
- The code was extremely difficult for new programmers to read or debug
- Removing the framework required touching every single file

The decision was made to strip NextFTC out entirely and rewrite everything in **plain iterative OpMode** style — the standard FTC SDK that every FTC programmer already knows.

---

## What Changed — April 6th, 2026 Update

### Autonomous 🤖
Two new fully written autos, both pathing-only for now with subsystem hooks ready to fill in:
- **BlueNineBallAuto** — 9 ball auto: preload → middle row → top row, built from PP visualizer
- **BlueTwelveBallAuto** — 12 ball auto: adapted from Cheick's original path coordinates, rewritten in plain iterative style with no NextFTC dependency. Uses a `BezierCurve` sweep on the middle row for better ball contact

Both autos use a clean enum state machine and a `waitFor()` timer helper instead of blocking `sleep()` calls.

### Fixed Turret Aiming — How It Works 🎯

The turret uses a two-stage aiming system to lock onto the goal.

**Stage 1 — Coarse aim (Pinpoint odometry)**
As soon as shoot mode is active, `followGoalOdometryPositional()` runs every loop tick. It takes the robot's current XY position and heading from Pinpoint, computes the angle to the goal, and sets the turret there. This gets the turret close fast but isn't precise enough on its own — odometry drift and mounting offsets mean it can be a few degrees off.

**Stage 2 — Fine tune (Limelight tx)**
Once the turret angle error drops below `fineTuneThresholdDeg` (default 5°), `fineTuneActive` flips to true and the Limelight turns on. From here, `getTx()` reads the horizontal offset in degrees between the camera crosshair and the goal AprilTag. If the tag is visible and the offset is larger than `fineTuneBangBangDeg` (default 0.5°), it nudges `angleOffset` by `fineTuneNudge` in the correct direction each tick. This offset carries forward into `followGoalOdometryPositional()` so the two stages stay in sync. If the tag disappears mid-match, it falls back to pure odometry automatically.

**Turret lock override**
Right Trigger toggles `turretLock`, which bypasses both stages entirely and holds the turret at the fixed `turretAngle` park position. Useful if tracking is fighting the driver or the tag is being blocked.

### Drivetrain 🚗
Rewrote drive math from scratch — cleaner mecanum calculations, removed unnecessary abstraction layers. Noticeably faster and more responsive than the NextFTC version.

### Spindexer Match Pattern Fix 🔄
Fixed a bug where the spindexer's shooting order could get out of sequence depending on intake order. The shooting sequence is now always correct regardless of which slots balls were loaded into.

### Subsystem Readmes 📖

Every subsystem now has its own README with a quickstart explanation, hardware notes, and tuning tips. Useful if anyone new needs to understand a specific mechanism without reading all the code.

---

## Controls, DriverStation, And Lookup Tables Links

[Controls Reference Link](https://docs.google.com/document/d/1A3iNGGY-CFJ41gOVWxeUdjWmHU4-4ooo/edit?usp=sharing&ouid=105244545883819724199&rtpof=true&sd=true)

[DriveStation Configs](https://docs.google.com/document/d/1f5MVQA72TZ1hu_ieV58IywkEtbo__3kph1g5qZGos4g/edit?usp=sharing)

[ShooterTable Spreadsheet](https://docs.google.com/spreadsheets/d/1oTGg8vLRNqRh52t9FsZwadkLCS6sTmQx/edit?usp=sharing&ouid=110208191089846695150&rtpof=true&sd=true)


---

## Subsystems (`org.firstinspires.ftc.teamcode.robot`)

#### `Drivetrain.java`
- Plain mecanum drive, `getInstance()` singleton
- `drive(y, x, rx)` — raw mecanum math
- Brake mode on all motors, **backLeft** reversed to match hardware

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
- Servo positions: leftUp=0.8, leftDown=0, rightUp=0.2, rightDown=1

#### `Spindexer.java`
- Three-position ball indexer with two NormalizedColorSensors
- Hardware: `spinServo`, `leftColorSensor`, `rightColorSensor`
- **Servo positions**: Intake (0.22, 0.51, 0.8) and Shoot (0.665, 0.08, 0.37)
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

#### `DistanceSensor.java` (`org.firstinspires.ftc.teamcode.Vision`)
- Wraps REV 2m distance sensor mounted at the shooter exit
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

### Autonomous (`org.firstinspires.ftc.teamcode.Auto`)
3 Autonomous pathings to be tuned and worked on next updates. 

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
- **`ShooterTables.java`** - Static table picker for Bilinear Lookup Table

---

## System Flow (TeleOp) 🔄

This robot operates as a **loop-driven, state-based system**.  
Each cycle of the loop reads inputs, updates internal state, and drives all subsystems accordingly.

---

### 🟢 Initialization Phase (`init` + `init_loop`)

On init:
- All subsystems and sensors are initialized (Drivetrain, Intake, Spindexer, Transfer, Turret, Pinpoint, Limelight, Distance Sensor)
- Transfer defaults to a safe position
- Match pattern detection is reset

During `init_loop`:
- Alliance is selected
- Field pattern (GPP / PGP / PPG) is detected and locked
- Sensor status is verified

This ensures the robot starts the match with correct configuration and field awareness.

---

### 🔄 Main Control Loop (Runs Every Tick)

Each loop cycle follows the same high-level flow: Driver Intent → State Updates → Subsystem Logic → Sensor Feedback → System Adjustments

---

### 🧠 State-Based Architecture

The robot does not directly map inputs to hardware.  
Instead, it maintains internal **states** such as:

- Intake active / inactive
- Shoot mode vs intake mode
- Flywheel state (off, spin-up, ready)
- Auto-shoot sequence state
- Transfer (flick) state
- Spindexer slot occupancy

These states are updated continuously and drive all behavior.

---

### ⚙️ Subsystem Coordination

All subsystems operate together based on current state:

- **Intake System**
    - Detects and confirms ball color using dwell timing
    - Assigns balls to spindexer slots
    - Tracks intake order

- **Spindexer**
    - Maintains internal model of ball positions
    - Rotates automatically to correct slot for intake or shooting
    - Handles full / empty transitions

- **Transfer (Flicker)**
    - Executes timed feed cycles into the shooter
    - Updates slot state after each shot

- **Turret + Shooter**
    - Continuously calculates target velocity and hood angle
    - Uses odometry and/or vision for distance estimation
    - Maintains aim using two-stage targeting (coarse + fine)

---

### 🎯 Targeting & Feedback Loop

Every tick:
- Robot position is updated via **Pinpoint**
- Vision data (Limelight) is used if available
- Shooter velocity + hood position are recalculated from lookup tables
- Turret adjusts toward goal in real time

This creates a continuous **feedback loop** for accurate shooting.

---

### 🔁 Automatic Behavior

The system handles transitions without driver intervention:

- Intake fills spindexer → automatically switches to shoot mode
- Shooting sequence continues until all balls are fired
- Once empty → system returns to intake mode

If vision is lost:
- System falls back to odometry-based targeting

---

### 🚗 Drivetrain

- Runs independently each loop using mecanum drive math
- Always responsive to driver input

---

### 🤖 Design Philosophy

- Loop-based, non-blocking architecture
- State machines control all sequencing
- Sensors continuously refine behavior
- Driver provides intent, system handles execution

The goal is a robot that is **consistent, predictable, and minimally dependent on driver micromanagement**.

---


## TODO — Next update or so..

### Autonomous
- [ ] Run Localization Test to verify Pinpoint is tracking correctly post-recode
- [ ] Test BlueNineBallAuto and BlueTwelveBallAuto driving only (no subsystems)
- [ ] Verify starting pose coordinates match actual field setup
- [ ] Wire in Intake + Spindexer calls into auto state machines
- [ ] Wire in Turret + Shooter calls into auto state machines
- [ ] Re-run all Pedro Pathing tuners to verify constants still hold on current bot

### Shooter & Data
- [ ] Re-tune bilinear interpolator tables from new robot position data (DataCollection.java)
- [ ] Verify hood and velocity lookup tables are accurate across full field range

### Future
- [ ] Red alliance autonomous
- [ ] Full 12-ball auto with subsystems wired in
- [ ] Shooting on the move (lead-target compensation using Pinpoint velocity)
---

## Future Goals

- **Shooting on the move** (Luke's request) — turret already auto-aims every tick, the remaining work is lead-target compensation: predicting where the goal will be relative to the robot by the time the ball arrives, then offsetting the aim angle and velocity accordingly. Ethan from 10195 got a slight version of this on their robot — the math is based on robot velocity from Pinpoint and known ball flight time.
- Full 12-ball autonomous (maybe even 26 ball)
- Red alliance autonomous

---

### Tuning OpModes (`org.firstinspires.ftc.teamcode.tuning`)

- **`FullTest.java`** — Complete mechanism test with Limelight. Standardized controls, dwell logic, jam retry. Alliance hardcoded Blue.
- **`DataCollection.java`** Full BiLinear Lookup Table testing.
- **`Outreach.java`** — Demo/event OpMode. Simplified for non-drivers. Origin of the dwell + auto-mode-switching logic now used everywhere.
- **`TestTurret.java`** — Isolated turret and encoder tuning
- **`TestShooter.java`** — Flywheel velocity tuning
- **`TestSpindexer.java`** — Spindexer position and color sensor tuning
- **`TestLimelight.java`** — AprilTag detection and distance testing
- **`TestHood.java`** — Hood servo position tuning
- **`TestDriveTrain.java`** - Simple Mecanum DriveTrain test to make sure wheels are working

---

## Need help? With General Tuning and learning how the bot works?
### Tuning & Maintenance Guide 🔧

#### **Re-Zeroing the Hood (`TestHood.java`)**
If the hood gear slips or the physical zero shifts:
1. Open **FTC Dashboard** and run `TestHood.java`.
2. Physically detach the hood from its driving gear.
3. In Dashboard, set `hoodPosition` to **0**.
4. Set `hoodPosition` to **0.5**.
5. While the servo is moving toward the 0.5 position, physically slide the hood back into the gears.
6. Verify range of motion in Dashboard.

#### **Re-Zeroing the Spindexer (`TestSpindexer.java`)**
If the Spindexer positions are misaligned with the intake or shooter:
1. Run `TestSpindexer.java` and open Dashboard.
2. Set all `intakePosition` and `shootPosition` variables to **0** in the `Spindexer` class via Dashboard.
3. Use the **Bumpers** on Gamepad 1 to cycle through positions.
4. With a partner's help, adjust the values in Dashboard until:
    - **Intake Mode**: Each slot is perfectly centered with the front of the intake.
    - **Shoot Mode**: Each slot is perfectly centered with the transfer forks.
5. Save these new values back into the `Spindexer.java` constants.

#### **Turret Tuning (`TestTurret.java`)**
- Use the Dashboard to play with PID values (`turretKp`, `turretKd`) and `maxPower`.
- Observe how the turret interacts with the Pinpoint and Limelight data.
- Check for oscillation or sluggishness and adjust coefficients accordingly.

#### **Shooter & Transfer Tuning**
- **Shooter**: Run `TestShooter.java` to verify flywheel RPM consistency and bang-bang threshold performance.
- **Transfer**: `TestTransfer.java` (if available) or `TestSpindexer.java` can be used via Dashboard to verify `leftUp`/`rightUp` and `leftDown`/`rightDown` positions are clearing the Spindexer slots cleanly.

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