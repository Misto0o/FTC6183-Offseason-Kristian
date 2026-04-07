# Robot Subsystems README
**Package:** `org.firstinspires.ftc.teamcode.robot`  
**Last Updated:** April 2026

---

## Drivetrain

**What it does:**  
Mecanum drivetrain. Takes joystick inputs (y, x, rx) and drives all 4 wheels with field-centric strafing math.

**Hardware:**  
`"fl"`, `"bl"`, `"fr"`, `"br"` — all DcMotorEx, all set to BRAKE mode.

### Key Methods

| Method | What it does |
|--------|-------------|
| `init(hardwareMap)` | Maps motors, sets directions and brake behavior |
| `drive(y, x, rx)` | Call every loop with joystick values (-1 to 1) |
| `setTurnSpeed(speed)` | Scales rotation (default 1.0) |

### Notes
- `backLeftMotor` is reversed; the others are forward — don't change this without re-testing strafing.
- `drive()` normalizes power so values never exceed 1.0.

---

## Intake

**What it does:**  
Single-motor intake. Spins to pull balls in, stop, or reverse to eject.

**Hardware:**  
`"intake"` — DcMotor, runs without encoder.

### Key Methods

| Method | What it does |
|--------|-------------|
| `init(hardwareMap)` | Maps motor, sets power to 0 |
| `on()` | Full power forward (1.0) |
| `idle()` | Stop (0.0) |
| `reverse()` | Full power reverse (-1.0) |

### Notes
- No variable speed — it's always full on or full off.
- Singleton via `Intake.INSTANCE`.

---

## Transfer

**What it does:**  
Two fork servos (left and right) that lift balls from the intake up to the spindexer.

**Hardware:**  
`"leftFork"`, `"rightFork"` — both Servos.

### Key Variables (dashboard tunable)

| Variable | Default | What it does |
|----------|--------|--------------|
| `leftUp / rightUp` | 0.8 / 0.2 | Servo positions when raised |
| `leftDown / rightDown` | 0.0 / 1.0 | Servo positions when lowered |

### Key Methods

| Method | What it does |
|--------|-------------|
| `initialize(hardwareMap)` | Maps both servos |
| `transferUp()` | Raises forks to pass ball |
| `transferDown()` | Lowers forks to receive ball |
| `isTransferDown()` | Returns true if both servos are down |

### Notes
- Servos are intentionally mirrored — this is correct.
- Forks are being reprinted so values may change.

---

## Spindexer

**What it does:**  
3-slot rotating disc that holds balls and uses color sensors to track them.

**Hardware:**  
`"spinServo"` — Servo  
`"leftColorSensor"`, `"rightColorSensor"` — NormalizedColorSensors

### Key Variables (dashboard tunable)

| Variable | Default | What it does |
|----------|--------|--------------|
| `plLower/plUpper` | 210 / 250 | Purple hue range (left) |
| `glLower/glUpper` | 145 / 170 | Green hue range (left) |
| `prLower/prUpper` | 200 / 240 | Purple hue range (right) |
| `grLower/grUpper` | 140 / 160 | Green hue range (right) |
| `intakePositionOne/Two/Three` | 0.22 / 0.51 / 0.8 | Intake positions |
| `shootPositionOne/Two/Three` | 0.665 / 0.08 / 0.37 | Shoot positions |

### How to use
1. Open up FTC dashboard and load `Spindexer` or `TestSpindexer.java`
2. Call `initialize(...)` in init
3. Call `periodic()` every loop
4. Use `freePosition()` / `filledPosition()`
5. Call `setColor(...)` after load/shoot
6. Switch modes with `setPositionType(...)`

### Notes
- Either sensor detecting color is enough (OR logic).
- Tune HSV ranges if detection is inconsistent.

---

## Turret

**What it does:**  
Aims and fires using flywheels, turret motor, and hood servo.

**Hardware:**  
`"shoot1"`, `"shoot2"` — DcMotorEx  
`"turret"` — DcMotor  
`"hood"` — Servo  
`"turretEncoder"` — AnalogInput

### Key Variables (dashboard tunable)

| Variable | Default | What it does |
|----------|--------|--------------|
| `turretVelocity` | 0 | Flywheel speed |
| `threshold` | 30 | Bang-bang deadband |
| `maxPower` | 0.45 | Max turret power |
| `turretKp / turretKd` | 0.01 / 0.001 | PD gains |
| `turretOffSet` | 250 | Encoder offset |
| `fineTuneThresholdDeg` | 5.0 | Switch to vision |
| `fineTuneNudge` | 0.05 | Vision adjustment |

### How to use
1. `initialize()` in init
2. `periodic()` every loop
3. `aimAtGoal(...)` every loop
4. `setVelocity(...)` to shoot
5. Use lookup helpers
6. `resetFineTune()` after shooting

### Notes
- Turret limited to 180°–360°.
- Flywheel motors spin opposite directions intentionally.

---

## Pinpoint

**What it does:**  
Tracks robot position using GoBilda odometry.

**Hardware:**  
`"pinpoint"` — GoBildaPinpointDriver

### Key Methods

| Method                  | What it does            |
|-------------------------|-------------------------|
| `init(hardwareMap)`     | Initializes odometry    |
| `periodic()`            | Updates position        |
| `getPosX() / getPosY()` | Returns position        |
| `getHeading()`          | Returns heading         |
| `updatePosition(pose)`  | Overrides position      |
| `resetPosAndIMU()`      | Resets position and IMU |


### Notes
- `periodic()` must run every loop.
- Call `updatePosition()` at start of autonomous.  