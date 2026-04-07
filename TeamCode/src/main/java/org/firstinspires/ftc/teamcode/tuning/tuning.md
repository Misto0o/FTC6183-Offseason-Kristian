# Tuning Quickstart
**Package:** `org.firstinspires.ftc.teamcode.tuning`
**Last Updated:** April 2026

## What This Is
A collection of isolated test opmodes used to tune, verify, and debug
individual subsystems. Run these before competition to confirm hardware
is working correctly before loading full teleop.

---

## driveTrainTest.java
Tests the drivetrain using the same `Drivetrain.java` math and directions
as teleop — what you see here is exactly what teleop will do.

**Hardware Used:** `"fl"`, `"bl"`, `"fr"`, `"br"`

**How To Use**
1. Run the opmode
2. Use DPad to spin one motor at a time and verify direction
3. For correct mecanum: DPad Up spins FL forward, confirm others match
4. If a motor spins wrong direction fix it in `Drivetrain.java` not here
5. Joystick drives full mecanum once directions are confirmed

| DPad | Motor |
|------|-------|
| Up | Front Left only |
| Down | Back Left only |
| Left | Front Right only |
| Right | Back Right only |

---

## TestHood.java
Bare minimum servo test — moves hood servo to whatever position is set
on the dashboard. Use this to find your physical min/max servo range.

**Hardware Used:** `"hood"`

**Key Variables**

| Variable | Default | What it does |
|----------|---------|--------------|
| `hoodPosition` | 0 | Servo position 0.0–1.0 |

**How To Use**
1. Run the opmode
2. Set `hoodPosition` on dashboard — try 0.0, 0.5, 1.0 to find range
3. Note the physical angles at each value
4. Use those values to populate `ShooterTables.java`

---

## TestLimelight.java
Verifies Limelight tag detection — shows every fiducial the camera
currently sees including raw IDs. Use this to confirm tag IDs match
your constants in `Limelight.java`.

**Hardware Used:** `"limelight"`, `"fl"`, `"bl"`, `"fr"`, `"br"`

**How To Use**
1. Run the opmode and drive in front of the field tags
2. Check `── RAW FIDUCIALS ──` section for detected IDs
3. Verify Blue goal shows ID 20, Red shows ID 24
4. Verify motif tags show 21 (GPP), 22 (PGP), 23 (PPG)
5. If IDs don't match update the constants in `Limelight.java`

**Known Issues / Notes**
- If `LLResult` is NULL the Limelight is not connected or wrong config name
- Raw fiducials section shows ALL tags — useful for debugging wrong IDs

---

## TestTransfer.java
Tests transfer servo range of motion. Tune the four position values
live on the dashboard without redeploying.

**Hardware Used:** `"leftFork"`, `"rightFork"`

**Key Variables**

| Variable | Default | What it does |
|----------|---------|--------------|
| `leftUp` | 0.8 | Left servo up position |
| `rightUp` | 0.2 | Right servo up position |
| `leftDown` | 0 | Left servo down position |
| `rightDown` | 1 | Right servo down position |

**How To Use**

1. Run the opmode
2. Press **Y** to toggle up/down and check range of motion
3. Press **A** to simulate a single flick (up then down)
4. Tune the four position values on dashboard until motion is clean
5. Values auto-save to `Transfer.java` via `@Config`

---

## TestSpindexer.java
Full spindexer isolation test — verifies servo positions, color sensor
reads, and the shoot-three sequence without the rest of the robot
running.

**Hardware Used:** `"spinServo"`, `"leftColorSensor"`, `"rightColorSensor"`,
`"leftFork"`, `"rightFork"`, intake motor

**How To Use**
1. Run the opmode — spindexer starts at POSITION_ONE in shoot mode
2. Use bumpers to step through positions and verify servo angles
3. Press **Circle** to turn intake on and check color sensor reads
4. Press **Cross** to run the full shoot-three sequence
5. Press **Square** to manually mark current slot as empty

| Button | Action |
|--------|--------|
| Left Bumper | Next position |
| Right Bumper | Previous position |
| DPad Left/Right/Down | Jump to position 1/2/3 |
| Triangle | Toggle shoot/intake mode |
| Circle | Toggle intake on/off |
| DPad Up | Single transfer flick |
| Cross | Auto shoot-three sequence |
| Square | Mark current slot empty |

---

## TestShooter.java
Tests flywheel velocity and hood position together. Use this to verify
the shooter reaches target velocity and that the bang-bang controller
is working correctly.

**Hardware Used:** `"shoot1"`, `"shoot2"`, `"hood"`, `"turretEncoder"`,
`"turret"`, `"pinpoint"`

**Key Variables**

| Variable | Default | What it does |
|----------|---------|--------------|
| `velocity` | 1000 | Target flywheel velocity |
| `hoodPosition` | 0.10 | Hood servo position |
| `turretAngle` | 270 | Fixed turret angle for testing |

**How To Use**
1. Run the opmode
2. Press **Square** to toggle flywheel on/off
3. Watch `Velocity (actual)` climb toward `Velocity (target)`
4. When `READY` shows press **X** to flick one ball
5. Adjust `velocity` and `hoodPosition` on dashboard and repeat

---

## TestTurret.java
Tests turret PD control and odometry-based goal tracking in isolation.

**Hardware Used:** `"turret"`, `"turretEncoder"`, `"pinpoint"`

**Key Variables**

| Variable | Default | What it does |
|----------|---------|--------------|
| `turretAngle` | 0 | Manual angle target when not tracking |

**How To Use**
1. Run the opmode — robot starts at `(8.5, 8.875)` heading `90°`
2. Manually set `turretAngle` on dashboard and verify turret moves correctly
3. Press **Cross** to toggle goal tracking on — turret should point at blue goal
4. Drive around and watch turret track
5. Press **Square** to reset odometry to starting position

---

## FullTest.java
Combined system test — intake, spindexer, turret tracking, and shooter
all running together without Pinpoint or MatchPattern. Used to verify
the full shoot cycle before running competition teleop.

**Hardware Used:** All shooter, intake, spindexer, transfer, drivetrain,
and Limelight hardware

**How To Use**
1. Run the opmode — alliance hardcoded to BLUE
2. Press **Triangle** to start intake
3. Drive under balls until spindexer is full — intake stops automatically
4. Press **Square** to spin up flywheel, wait for rumble
5. Press **Square** again to fire one ball
6. Press **Cross** to do a single manual flick without waiting for rumble

**Known Issues / Notes**
- No Pinpoint — turret tracks via `followGoalOdometryPositional` but
  without odometry updates position drifts over time
- No MatchPattern — shot order is just first filled slot, no pattern logic
- Use this to verify hardware works, use teleop for actual match prep

---

## DataCollection.java
Shooter tuning opmode. Drive to a spot, tune velocity and hood on the
dashboard until shots are consistent, press Cross to log the data point.
Logged points are ready to paste directly into `ShooterTables.java`.

See the comment block at the top of `DataCollection.java` for full
step-by-step instructions.

**Key Variables**

| Variable | Default | What it does |
|----------|---------|--------------|
| `shootVelocity` | 1000 | Flywheel target velocity to tune |
| `hoodPosition` | 0.10 | Hood servo position to tune |

**Known Issues / Notes**
- Logged points only show on Driver Station — take a photo or write into spreadsheet
  before stopping the opmode or they will be lost
- Red alliance data collection not yet implemented — do blue first

## TestPinpoint.java
Tests Pinpoint odometry — verifies position tracking and heading are
accurate before running any auto or turret tracking that depends on it.

**Hardware Used:** `"pinpoint"`, `"fl"`, `"bl"`, `"fr"`, `"br"`

**How To Use**
1. Place robot at a known field position and run the opmode
2. Drive forward — Y should increase. Strafe right — X should increase
3. Rotate CCW — heading should increase
4. Drive a full square and verify robot returns close to (0, 0)
5. Press **A** to reset position and heading to zero at any time

**Sanity Checks**

| Action | Expected |
|--------|----------|
| Drive forward | Y increases |
| Strafe right | X increases |
| Rotate CCW | Heading increases |
| Full square path | Returns near (0, 0) |

**Key Notes**
- Heading comes from Pinpoint's onboard IMU, not the Control Hub IMU
- If drift is bad on the square test, check pod contact with the ground
  and verify offsets in `Pinpoint.java` (4.5" strafe, −7.125" forward)
- Fix encoder direction issues in `Pinpoint.java`, not here

**Known Issues / Notes**
- If telemetry shows 0,0 and never updates, Pinpoint is not connected
  or the hardware name `"pinpoint"` doesn't match your config