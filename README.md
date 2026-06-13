# FTC 6183 Loki — Robot Recode 🚀

This repository contains the completely refactored robot software for **FTC Team 6183 Loki**. 

During this 5-month offseason project, the codebase was rebuilt from the ground up, transitioning from a third-party command framework (NextFTC) to a clean, iterative **FTC SDK** implementation. The goal was to create a robust, readable, and highly maintainable foundation for current and future team members.

---

## 🏗️ Project Overview

- **Architecture:** Iterative OpMode (Plain FTC SDK)
- **Status:** Active Offseason Development / Refactor Complete
- **Core Goal:** Reliability, Debuggability, and Ease of Training
- **Lead Developer:** Kristian
- **Contributor/Mentor:** Luke

---

## 🌟 Key Features

- **Custom Mecanum Drivetrain:** Optimized math for responsive and precise movement.
- **Two-Stage Turret Aiming:** Combines **Pinpoint Odometry** for coarse targeting with **Limelight 3A** for high-precision fine-tuning.
- **Bilinear Interpolation:** Dynamic lookup tables for shooter velocity and hood angles based on real-time field position.
- **State-Machine Driven Subsystems:** Robust, non-blocking control logic for the Spindexer, Intake, and Transfer systems.
- **Pattern Awareness:** Integrated logic to detect and sequence shots according to the field's AprilTag motif (GPP/PGP/PPG).
- **Comprehensive Documentation:** Every package includes dedicated READMEs and tuning guides.

---

## 📂 Repository Structure

- [**`TeamCode/`**](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md) — The heart of the robot software.
    - [`robot/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/Subsystems.md) — All hardware subsystems (Drivetrain, Turret, Spindexer, etc.).
    - [`teleop/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/Quickstart.md) — Competition and manual control OpModes.
    - [`Auto/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/) — Pathing-only and fully integrated autonomous routines.
    - [`Vision/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Vision/Vision.md) — Limelight and Distance Sensor wrappers.
    - [`Utils/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Utils/Utils.md) — Math helpers, Interpolators, and Pattern Logic.
    - [`tuning/`](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/tuning.md) — Isolated testing and calibration tools.

---

## 🛠️ Getting Started

1. **Clone the Repo:** Ensure you have the latest code.
2. **Review the [Main TeamCode README](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md):** Detailed technical breakdown of the systems.
3. **Hardware Configuration:** Matches the configuration name `Loki` (or as specified in the `robot/` subsystem files).
4. **Tuning:** Use the OpModes in the `tuning` package to verify sensor thresholds and servo positions before running TeleOp.

---

## 📝 Contributions

This project is a labor of love for Team 6183. If you are a team member looking to contribute, please check the package-specific READMEs for logic flow and "How-To" guides for each mechanism.
