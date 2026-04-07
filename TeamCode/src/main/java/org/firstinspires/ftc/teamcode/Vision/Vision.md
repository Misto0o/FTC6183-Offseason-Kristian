# Vision Package
**Package:** `org.firstinspires.ftc.teamcode.Vision`
**Last Updated:** April 2026

## What This Is
Sensor wrapper classes for the Limelight 3A camera and the REV 2m distance sensor.
Used by `Turret.java` for fine-tune aiming and by the transfer system to confirm
ball presence at the shooter exit. Neither class is an OpMode.

---

## Limelight.java
Singleton wrapper around the Limelight 3A AprilTag camera. Handles all tag
detection — turret fine-tune aiming, distance estimation, and obelisk pattern
detection. All methods return safe defaults if the Limelight is unplugged or
the tag isn't visible, so the robot will never crash from a missing camera.

**Hardware Name:** `"limelight"`

**Tag IDs**

| Constant | ID | Meaning |
|----------|----|---------|
| `BLUE_GOAL_ID` | 20 | Blue alliance goal |
| `RED_GOAL_ID` | 24 | Red alliance goal |
| `GPP_PATTERN_ID` | 21 | Green-Purple-Purple motif |
| `PGP_PATTERN_ID` | 22 | Purple-Green-Purple motif |
| `PPG_PATTERN_ID` | 23 | Purple-Purple-Green motif |

**Key Methods**

| Method | What it does |
|--------|--------------|
| `initialize(hwMap)` | Inits hardware — wrapped in try/catch, safe if unplugged |
| `start()` | Starts the Limelight pipeline |
| `stop()` | Stops the Limelight pipeline — call to save resources |
| `getTx(tagId)` | Returns horizontal offset in degrees to the target tag. Returns `0` if tag not seen |
| `distanceFromTag(tagId)` | Returns estimated distance in inches to the target tag. Returns `0` if not seen |
| `patternFromObelisk()` | Returns the pattern tag ID (21/22/23) seen on the obelisk. Returns `-1` if none |
| `getRawResult()` | Raw `LLResult` for debugging — shows everything the camera currently sees |

**How It's Used**
- `Turret.java` calls `getTx(BLUE_GOAL_ID)` every tick during fine-tune stage to nudge aim
- `MatchPattern.java` calls `patternFromObelisk()` during `init_loop()` to lock the field motif
- `distanceFromTag()` adds a fixed `+16` inch offset to both X and Z — this accounts for
  camera mounting position, do not remove it

**Known Issues / Notes**
- Poll rate is set to 100Hz on init — do not lower this or aiming will lag
- If `limelight` is null (unplugged), every method returns a safe default and logs nothing
- Always call `stop()` after pattern is locked to free up resources during the match

---

## DistanceSensor.java
Wrapper around the REV 2m distance sensor mounted at the shooter exit.
Used to confirm whether a ball is present in the transfer path before
or after a shot. Returns `999cm` on any error so it never falsely
triggers a ball-present reading.

**Hardware Name:** `"distanceSensor"`

**Tunable Constants** (via FTC Dashboard)

| Constant | Default | Meaning |
|----------|---------|---------|
| `BALL_MIN_CM` | 3.0 | Minimum distance to count as ball present |
| `BALL_MAX_CM` | 15.0 | Maximum distance to count as ball present |

**Key Methods**

| Method | What it does |
|--------|--------------|
| `init(hwMap)` | Initializes the sensor from hardware map |
| `getDistanceCM()` | Raw distance reading in cm. Returns `999` on null/NaN/infinite |
| `isBallPresent()` | Returns `true` if distance is within `BALL_MIN_CM`–`BALL_MAX_CM` |
| `isClear()` | Returns `true` if no ball is present — inverse of `isBallPresent()` |

**Known Issues / Notes**
- If the sensor is unplugged, `getDistanceCM()` returns `999` — this means
  `isBallPresent()` will always be `false` and `isClear()` always `true`
- Tune `BALL_MIN_CM` and `BALL_MAX_CM` on Dashboard if detection is triggering
  too early or missing balls entirely
- Sensor has a minimum reliable range — do not set `BALL_MIN_CM` below `2.0`