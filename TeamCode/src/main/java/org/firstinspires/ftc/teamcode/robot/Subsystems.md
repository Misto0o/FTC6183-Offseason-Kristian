# Robot Subsystems ⚙️

**Package:** `org.firstinspires.ftc.teamcode.robot`  
**Last Updated:** May 2026

This package contains the hardware abstraction layers for all robot mechanisms. Each class handles the low-level motor/servo logic for a specific subsystem using the Singleton pattern for global state consistency.

---

## 🏎️ Drivetrain
**Class:** `Drivetrain.` (Singleton)

**Purpose:**  
Standard Mecanum drivetrain with field-centric capabilities. Handles joystick normalization and motor power distribution across four motors.

**Hardware Map:**
- `fl` / `bl` / `fr` / `br` (DcMotorEx)
- Zero Power Behavior: `BRAKE`
- Motor Directions: FL=FORWARD, BL=REVERSE, FR=FORWARD, BR=FORWARD

**Key Methods:**
- `drive(y, x, rx)`: Main drive function. Normalizes inputs by max denominator to prevent over-command.
- `setTurnSpeed(speed)` / `getTurnSpeed()`: Scale rotational sensitivity.

**Maintenance Note:** `backLeftMotor` is physically reversed. If the robot strafes incorrectly, verify this direction first.

---

## 📥 Intake
**Class:** `Intake.` (Singleton)

**Purpose:**  
Manages the front intake rollers for picking up balls from the floor.

**Hardware Map:**
- `intake` (DcMotor, RUN_WITHOUT_ENCODER)

**Key Methods:**
- `on()`: Full forward power (1.0).
- `idle()`: Stop intake (0.0).
- `reverse()`: Reverse intake to clear jams (-1.0).

---

## 🏗️ Transfer
**Class:** `Transfer.` (Singleton)

**Purpose:**  
A dual-servo fork mechanism that transfers balls from the floor intake into the rotating Spindexer. Includes aggressive pull-down to prevent jamming.

**Hardware Map:**
- `leftFork` / `rightFork` (Servos)

**Tunable Constants:**
- `leftUp` / `rightUp`: High position for spindexer delivery (~0.4 / 0.6).
- `leftDown` / `rightDown`: Low position for floor intake (0.0 / 1.0).

**Key Methods:**
- `transferUp()`: Raise forks to delivery position.
- `transferDown()`: Lower forks to rest position.
- `transferDownAggressive()`: **NEW** — Pulls forks beyond [0, 1] servo limits to fight friction/gravity and prevent jamming.
- `isTransferDown()`: Check if both forks have settled at rest position.

**Important:** Use `transferDownAggressive()` in state machines to prevent fork blockage of the Spindexer.

---

## 🔄 Spindexer
**Class:** `Spindexer.` (Singleton)

**Purpose:**  
A 3-slot rotating magazine that tracks ball color and occupancy using dual color sensors. Switches between INTAKE and SHOOT modes for different servo positions.

**Hardware Map:**
- `spinServo` (Servo)
- `leftColorSensor` / `rightColorSensor` (NormalizedColorSensor)

**Color Detection:**
- Purple Hue Range: 200–250° (both sensors)
- Green Hue Range: 140–170° (both sensors)
- **Detection Logic:** OR across both sensors (only one needs to see the color).
- **Tuning:** If color detection fails, adjust HSV ranges using `SensorColor.`.

**Key Methods:**
- `setToPosition(Position)`: Rotate to a slot based on current mode (INTAKE or SHOOT).
- `freePosition()`: Find next empty slot (returns index 0-2 or -1).
- `filledPosition()`: Find next occupied slot for shooting.
- `readCurrentColor()`: Read both sensors and return detected color.
- `setColor(Position, DetectedColor)`: Manually stamp a slot with a color.
- `periodic()`: Update `full` and `empty` flags. **Must be called every loop.**

**Intake Flow:**
1. Intake spins, ball enters current slot.
2. `dwellTimer` (0.3s) allows ball to settle.
3. Color sensors read Hue → slot is "stamped" with color.
4. Rotates to next free slot or enters SHOOT mode if full.

---

## 🎯 Turret
**Class:** `Turret.` (Singleton)

**Purpose:**  
The primary scoring mechanism. Handles flywheel velocity control, turret rotation tracking, and hood angle adjustment.

**Hardware Map:**
- `shoot1` / `shoot2` (DcMotorEx, FLOAT behavior)
- `turret` (DcMotor, RUN_WITHOUT_ENCODER)
- `hood` (Servo)
- `turretEncoder` (AnalogInput, absolute 0-3.3V)

**Tunable Constants:**
- `turretOffSet = 350`: **CALIBRATED** — Maps encoder voltage to degrees. Do not change unless re-calibrating.
- `maxPower = 0.4`: Limits turret rotation speed to prevent overshoot.
- `turretKp = 0.03` / `turretKd = 0.002`: PD coefficients for smooth tracking.
- `threshold = 15`: Acceptable velocity error for flywheels (ticks/sec).

**Goal Coordinates (0-144 field system):**
- `BLUE_GOAL_X = 0, BLUE_GOAL_Y = 144`
- `RED_GOAL_X = 144, RED_GOAL_Y = 144`

**Control Systems:**

*Flywheels:* Bang-Bang control
- Below threshold: Full power (1.0)
- Within threshold: Maintain current power
- Above threshold: Cut power (0.0)
- **Rationale:** Fastest recovery vs. traditional PID.

*Rotation:* PD position control
- Uses shortest-path error calculation to prevent oscillation.
- Encoder-based feedback with analog voltage mapping.

*Aiming:* Two-stage odometry + vision
- Primary: `aimAtGoal()` uses Pinpoint odometry to calculate turret angle.
- Formula: `turretAngle = atan2(goalY - robotY, goalX - robotX) - robotHeading`
- All angles normalized to [0, 360) for servo compatibility.

**Key Methods:**
- `aimAtGoal(Alliance, goalId)`: Auto-aim turret at goal using odometry.
- `setVelocity(v)`: Command flywheel RPM (ticks/sec).
- `getVelocity()`: Read current flywheel speed.
- `isSettled()`: Check if turret is on-target AND flywheel at speed.
- `setHoodPosition(pos)`: Command hood servo [0.0–1.0].
- `setToAngle(angle)`: Command turret rotation [0–360°].
- `getTurretAngle()`: Read actual turret angle from encoder.
- `calibrateTurretZero()`: Re-calibrate encoder offset (expert use only).
- `periodic()`: Update all control loops. **Must be called every loop.**

---

## 📍 Pinpoint
**Class:** `Pinpoint.` (Singleton)

**Purpose:**  
Wraps the GoBilda Pinpoint Computer for high-accuracy dead-reckoning position tracking (X, Y, Heading) in field coordinates.

**Hardware Map:**
- `pinpoint` (GoBildaPinpointDriver)

**Configuration:**
- Encoder Resolution: GoBilda 4-Bar Pods
- Encoder Directions: FORWARD, REVERSED
- Physical Offsets: 4.5" X, -7.125" Y

**Key Methods:**
- `updatePosition(Pose2D)`: Manually set robot pose. Call once at match start.
- `getPosX()` / `getPosY()` / `getHeading()`: Read current position.
- `getHeadingNormalized()`: Heading in [-180, 180] range.
- `relocalizeHeadingFromLimelight(xOffset)`: **NEW** — Correct heading drift using Limelight tag detection without affecting X/Y.
- `periodic()`: Update odometry from encoders. **Must be called every loop.**

**Drift Characteristics:**
- Heading: Can drift 5–10° over a match. Use Limelight auto-relocalization to correct.
- Position: Encoder drift is slow; reset to corner occasionally if critical.

**Integration with Limelight:**
When the robot stops near a goal tag, Limelight automatically detects it and corrects the heading. This minimizes manual resets during matches.

---

## 🔧 Singleton Pattern

All subsystem classes use the Singleton pattern to ensure only one instance exists:

```
public static final ClassName INSTANCE = new ClassName();
private ClassName() {}  // Private constructor prevents new instances
```

**Usage:**
```
Turret.INSTANCE.aimAtGoal(Aliance.BLUE, 20);
Spindexer.INSTANCE.periodic();
Pinpoint.INSTANCE.getPosX();
```

---

## ⚡ Lifecycle Summary

**Initialization (OpMode.init()):**
1. Call `init()` on all subsystems.
2. Transfer forks to down position.
3. Spindexer in INTAKE mode.

**Loop (OpMode.loop()):**
1. Read gamepad inputs.
2. Update Pinpoint, Drivetrain, Turret.
3. Call `periodic()` on all subsystems.
4. Update telemetry.

**Important:** Every subsystem's `periodic()` method must be called every loop for smooth operation.

---

## 🐛 Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Robot strafes backwards | Motor direction issue | Check `Drivetrain` motor directions |
| Forks jam Spindexer | Friction/gravity | Use `transferDownAggressive()` |
| Turret won't aim correctly | Bad encoder offset or goal coords | Check `turretOffSet = 350` and goal coords (0,144) |
| Color sensors miss balls | HSV ranges too narrow | Adjust `plUpper/plLower/glUpper/glLower` |
| Odometry drifts mid-match | Encoder drift | Use Limelight auto-relocalize when near goal |
| Flywheel overshoots | PD tuning | Decrease `turretKp` or increase `threshold` |