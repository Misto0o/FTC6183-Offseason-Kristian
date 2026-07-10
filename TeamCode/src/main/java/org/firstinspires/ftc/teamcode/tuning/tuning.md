# Tuning & Calibration Guide 🔧
**Package:** `org.firstinspires.ftc.teamcode.tuning`
**Last Updated:** June 2026

This package contains isolated OpModes used to calibrate hardware constants. These should be run whenever physical changes are made to the robot (e.g., re-stringing, gear replacement, or sensor relocation).

---

## 🏎️ Drivetrain Test
**OpMode:** `driveTrainTest.java`
- **Purpose:** Verify motor directions and mecanum strafing logic.
- **Controls:** Use the DPad to isolate single motors (Up: FL, Down: BL, Left: FR, Right: BR).
- **Goal:** Ensure "Forward" on the joystick moves all wheels forward.

## 🏹 Turret & Aiming
**OpMode:** `TestTurret.java`
- **Purpose:** Tune the PD coefficients (`turretKp`, `turretKd`), verify the absolute encoder offset (`turretOffSet`), and record the turret's physical rotation limits.
- **Controls:**
    - **Square:** Toggle goal tracking (aims at the Blue goal using `goalX`/`goalY`, set via FTC Dashboard).
    - **Circle:** Return turret to parked (270°).
    - **Triangle:** Recalibrate parked position (same as Teleop's calibrate-to-parked).
    - **Left Bumper:** Toggle motor power. Turn motor OFF to push the turret by hand and find its real physical limits without the PD loop fighting you.
    - **DPad Right / Left:** Record the current turret angle as the right/left physical limit (do this while motor is OFF and you've pushed it to the hard stop).
    - **DPad Up:** Reset odometry to the Blue starting corner.
- **Procedure:** Drive the robot and check if the turret stays locked on the Blue goal with tracking enabled. Separately, turn the motor off, physically push the turret to each hard stop, and record both limits. Use these to sanity-check `aimAtGoal()`'s `[180, 360]` clamp in `Turret.java`.

## 🌀 Spindexer Calibration
**OpMode:** `TestSpindexer.java`
- **Purpose:** Fine-tune the servo positions for "Intake" and "Shoot" modes.
- **Goal:** Slots must align perfectly with the transfer forks in shoot mode to prevent jams. Note: `Teleop.java` now also gates firing on a `SPINDEXER_SETTLE_SEC` timer (spindexer must finish its move before the fork is allowed up) — if jams persist even after this calibration is dialed in, that timer may need lengthening rather than the slot positions themselves.

## 🔄 Transfer & Forks
**OpMode:** `TestTransfer.java`
- **Purpose:** Tune `leftUp`/`rightUp` and `leftDown`/`rightDown` positions.
- **Goal:** Smooth motion without binding against the Spindexer plate.

## 🚀 Shooter & Data Collection
**OpMode:** `DataCollection.java`
- **Purpose:** The most important tuning tool. Drive to various distances, find the "sweet spot" RPM and Hood angle, and log the data.
- **Outcome:** These points are pasted into `ShooterTablesV2.java` to build the robot's "aim assist" map.

## 🎯 Limelight / Localization Test
**OpMode:** `TestLimelight.java`
- **Purpose:** Verify AprilTag visibility and check vision-based localization accuracy against known field positions.
- **Note:** Uses **MegaTag1** (`getMT1Pose()` / `getMT1Yaw()` / `getMT1PoseRaw()`), not MegaTag2 — MT1 solves rotation fresh from the tag every frame, so no robot-orientation feed is required for the pose readout to be valid.
- **Procedure:** Park the robot at 2+ different known field spots and compare `MT1 raw inches` / `MT1 converted` telemetry against the robot's actual position at each spot. One spot checking out is not sufficient proof - the conversion math (axis swap + flip + corner-origin shift) needs to hold at multiple locations before it's trusted in a match.

---

## 🛠️ Calibration Workflow
1. **Mechanical Check:** Ensure all set screws are tight.
2. **Sensor Check:** Run `TestLimelight.java` to verify target visibility and validate the MT1 pose conversion at 2+ known field spots.
3. **Turret Check:** Run `TestTurret.java` - disable the motor (Left Bumper) to find and record physical rotation limits, then re-enable and verify goal tracking holds steady.
4. **Servo Check:** Run `TestHood.java` to find the physical limits of the hood.
5. **Integration Check:** Run `FullTest.java` to see all systems working together before jumping into a full match.