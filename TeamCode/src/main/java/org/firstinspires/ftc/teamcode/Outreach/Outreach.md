# Outreach & Demo Mode 🤝
**Package:** `org.firstinspires.ftc.teamcode.Outreach`  
**Last Updated:** April 2026

## Overview
`Outreach.java` is a simplified OpMode designed for public demonstrations and community events. It removes complex dependencies like Odometry and Limelight to ensure the robot remains easy to operate for non-drivers while maintaining the core "Intake and Shoot" experience.

---

## 🎮 How To Use
1. **Initialize:** Select `Outreach` from the TeleOp menu.
2. **Intake (Triangle):** Drive over balls to fill the Spindexer. The intake stops automatically when the 3 slots are full.
3. **Spin Up (Square):** Press once to start the flywheels. The controller will rumble 3 times when the target RPM is reached.
4. **Fire (Square):** Press again to fire a single ball.
5. **Auto-Fire (Cross):** Rapidly sequence all 3 balls into the goal.
6. **Kill Switch (DPad Left):** Immediately stops all motors and resets the state machines.

---

## ⚙️ Key Configurations
- **Velocity Control:** DPad Up/Down toggles between 3000 RPM (Far) and 1000 RPM (Near).
- **Turret:** Fixed at -90° (static forward position).
- **No Pattern Logic:** The robot fires balls in the order they were intaked, ignoring color sorting.

---

## 🛠️ Internal State Machines

### ShooterState
| State | Behavior |
|-------|----------|
| 0 | Off |
| 1 | Spinning up — monitoring RPM |
| 2 | Ready — target velocity achieved |

### FlickState (Manual)
| State | Behavior |
|-------|----------|
| IDLE | Forks lowered |
| WAIT_UP | Moving to delivery position |
| WAIT_DOWN | Returning to rest |

---

## ⚠️ Notes
- Designed for safety: Reduced power and simplified controls.
- Useful for training new members on basic robot interaction before moving to the full Competition TeleOp.
