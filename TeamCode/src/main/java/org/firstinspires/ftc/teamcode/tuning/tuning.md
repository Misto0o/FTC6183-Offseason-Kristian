# Tuning & Calibration Guide 🔧
**Package:** `org.firstinspires.ftc.teamcode.tuning`
**Last Updated:** April 2026

This package contains isolated OpModes used to calibrate hardware constants. These should be run whenever physical changes are made to the robot (e.g., re-stringing, gear replacement, or sensor relocation).

---

## 🏎️ Drivetrain Test
**OpMode:** `driveTrainTest.java`
- **Purpose:** Verify motor directions and mecanum strafing logic.
- **Controls:** Use the DPad to isolate single motors (Up: FL, Down: BL, Left: FR, Right: BR).
- **Goal:** Ensure "Forward" on the joystick moves all wheels forward.

## 🏹 Turret & Aiming
**OpMode:** `TestTurret.java`
- **Purpose:** Tune the PD coefficients (`turretKp`, `turretKd`) and verify the absolute encoder offset (`turretOffSet`).
- **Procedure:** Drive the robot and check if the turret stays locked on the Blue goal.

## 🌀 Spindexer Calibration
**OpMode:** `TestSpindexer.java`
- **Purpose:** Fine-tune the servo positions for "Intake" and "Shoot" modes.
- **Goal:** Slots must align perfectly with the transfer forks in shoot mode to prevent jams.

## 🔄 Transfer & Forks
**OpMode:** `TestTransfer.java`
- **Purpose:** Tune `leftUp`/`rightUp` and `leftDown`/`rightDown` positions.
- **Goal:** Smooth motion without binding against the Spindexer plate.

## 🚀 Shooter & Data Collection
**OpMode:** `DataCollection.java`
- **Purpose:** The most important tuning tool. Drive to various distances, find the "sweet spot" RPM and Hood angle, and log the data.
- **Outcome:** These points are pasted into `ShooterTables.java` to build the robot's "aim assist" map.

---

## 🛠️ Calibration Workflow
1. **Mechanical Check:** Ensure all set screws are tight.
2. **Sensor Check:** Run `TestLimelight.java` to verify target visibility.
3. **Servo Check:** Run `TestHood.java` to find the physical limits of the hood.
4. **Integration Check:** Run `FullTest.java` to see all systems working together before jumping into a full match.
