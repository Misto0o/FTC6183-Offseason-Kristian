# Vision & Perception 👁️
**Package:** `org.firstinspires.ftc.teamcode.Vision`  
**Last Updated:** April 2026

This package contains wrappers for the robot's primary sensors used for targeting, pattern detection, and ball tracking.

---

## 🟢 Limelight
**Class:** `Limelight.java` (Singleton)

**Hardware:** Limelight 3A

**Responsibilities:**
- **Target Tracking:** Provides `tx` (horizontal offset) for the Turret fine-tuning stage.
- **Distance Estimation:** Calculates distance to AprilTags for automated shooter adjustments.
- **Pattern Detection:** Identifies the field's randomization motif (GPP/PGP/PPG) from the obelisk.

**Key Safety Feature:** All methods are wrapped in try-catch blocks or null guards. If the Limelight is disconnected, the robot will fall back to Odometry targeting without crashing.

---

## 📏 Distance Sensor
**Class:** `DistanceSensor.java`

**Hardware:** REV 2m Laser Distance Sensor

**Responsibilities:**
- Mounted at the shooter exit.
- Confirms when a ball has successfully left the robot.
- Triggers the "Next Ball" state in the Auto-Shoot sequence.

**Tuning:**
- `BALL_MIN_CM` and `BALL_MAX_CM` define the detection window. If the sensor misreads, adjust these values on the FTC Dashboard.

---

## 📊 Ramp Scanner
**Class:** `RampScanner.java`

**Responsibilities:**
- Scans the field's neutral ramp to identify available ball colors.
- Feeds data to the `PatternStrategy` to decide if the robot should intake more balls to complete a pattern bonus.
