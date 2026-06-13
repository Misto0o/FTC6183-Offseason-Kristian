# Utilities Package 🛠️
**Package:** `org.firstinspires.ftc.teamcode.Utils`
**Last Updated:** April 2026

This package contains mathematical helpers, game-specific logic, and shared constants used across the entire codebase.

---

## 📐 Interpolator
**Purpose:**  
A 2D lookup table using Bilinear Interpolation. It takes discrete data points (like "at position X,Y, the shooter needs power Z") and calculates the exact requirement for any point in between.

**Key Feature:**  
Ensures the shooter and hood automatically adjust to the robot's distance and angle relative to the goal without manual driver input.

---

## 📊 ShooterTables
**Purpose:**  
Static storage for calibrated shooter data. It populates the `Interpolator` instances with real-world testing data for both Blue and Red alliances.

**Contents:**
- Flywheel RPM targets.
- Hood servo position targets.

---

## 🎯 MatchPattern
**Purpose:**  
Handles the detection of the randomization pattern (GPP, PGP, PPG) during the `init_loop`.

**Logic:**
- Uses the Limelight to scan the obelisk.
- "Locks" the pattern once confirmed to prevent accidental re-scanning during the match.
- Shuts down the Limelight pipeline after locking to save processing power.

---

## 🧠 PatternStrategy
**Purpose:**  
The "Brain" of the scoring system. It compares what is currently in the Spindexer and on the field ramp against the required Match Pattern.

**Decision Output:**
- Can we score the full pattern?
- What color sequence should we fire?
- Do we need to intake more balls from the ramp first?

---

## 🎨 SensorColor
**Purpose:**  
A calibration tool for the Spindexer's color sensors. It provides live HSV (Hue, Saturation, Value) feedback to help tune the detection thresholds for purple and green balls.
