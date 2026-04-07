# Outreach.java
**Package:** `org.firstinspires.ftc.teamcode.Outreach`
**Last Updated:** April 2026

## What This Does
Simplified OpMode for demo and outreach events. Designed to be easy for
non-drivers to pick up and use — no odometry, no Limelight, no pattern
detection. Just intake, shoot, and drive.

## State Machines

## ShooterState (int)

| Value | Meaning |
|-------|---------|
| 0     | Off |
| 1     | Spinning up — waiting to reach target velocity |
| 2     | Ready — press Square to fire |

---

## ShootCycleState (Cross auto-shoot)

| State         | Meaning |
|---------------|---------|
| IDLE          | Not running |
| SET_POS       | Moving spindexer to next ball |
| WAIT_POS      | Waiting for spindexer to settle |
| TRANSFER_UP   | Flicking transfer up |
| TRANSFER_DOWN | Flicking transfer back down |
| MARK_EMPTY    | Marking slot as empty, advancing to next ball |

---

## FlickState (manual single shot)

| State     | Meaning |
|-----------|---------|
| IDLE      | Transfer down, ready |
| WAIT_UP   | Transfer moving up, waiting 0.5s |
| WAIT_DOWN | Transfer moving back down, waiting 0.5s |

## Key Variables
| Variable | Default | What it does |
|----------|---------|--------------|
| `shootVelocity` | 3000 | Target flywheel velocity |
| `intakeDwellSec` | 0.3 | How long to wait at each spindexer slot before reading color |

## How To Use
1. Power on robot and select **Outreach** from the TeleOp menu
2. Press PLAY
3. Hand the controller to the demo participant
4. They press **Triangle** to turn on intake and drive under balls
5. Spindexer fills automatically — intake stops when full
6. Press **Square** to spin up the flywheel — controller rumbles 3 times when ready
7. Press **Square** again to fire one ball — repeats until all 3 are fired
8. Or press **Cross** to auto-fire all 3 balls in sequence
9. Press **DPad Left** at any time to kill everything and reset

## Known Issues / Notes
- Turret is locked at -90° no tracking
- DPad Up sets velocity to 3000, DPad Down sets to 1000 — useful if
  shooting from different distances at an event
- No pattern detection — balls are fired in slot order regardless of color
- If something jams press DPad Left to fully reset all state machines