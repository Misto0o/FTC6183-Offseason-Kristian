# Teleop.java
**Package:** `org.firstinspires.ftc.teamcode.teleop`
**Last Updated:** April 2026

## What This Does
Main competition teleop. Handles intake, spindexer color tracking, turret
aiming via odometry + Limelight fine-tune, hood auto-adjustment from lookup
tables, pattern-aware shot ordering, and auto-shoot with distance sensor
confirmation.


## State Machines

### squareState (flywheel)
| Value | Meaning |
|-------|---------|
| 0 | Off |
| 1 | Spinning up — waiting to reach target velocity |
| 2 | Ready — controller rumbles once, press Square to shoot |

### FlickState (transfer)
| State | Meaning |
|-------|---------|
| IDLE | Transfer down, ready |
| WAIT_UP | Transfer moving up, waiting 0.5s |
| WAIT_DOWN | Transfer moving back down, waiting 0.5s |

### AutoShootState (Cross auto-shoot)
| State | Meaning |
|-------|---------|
| IDLE | Not running |
| CONFIRM_SHOT | Waiting for distance sensor to confirm ball left shooter |
| WAIT_REFIRE | Shot not confirmed — waiting before refiring |
| NEXT_BALL | Shot confirmed — advancing to next ball |

### RescanState (triangle second press)
| State | Meaning |
|-------|---------|
| IDLE | Not scanning |
| MOVING | Spindexer moving to next slot, waiting 150ms to settle |
| READING | Reading color sensor, advancing to next slot |

## Key Variables
| Variable | Default | What it does                                         |
|----------|---------|------------------------------------------------------|
| `shootVelocity` | -1      | Use the lookup table value uncapped.                 |
| `turretAngle` | 123.5   | Resting turret angle when not in shoot mode          |
| `intakeDwellSec` | 0.3     | How long to wait at each slot before stamping color  |
| `SHOT_CONFIRM_TIMEOUT` | 1.5     | Seconds before auto-shoot assumes a miss and refires |
| `REFIRE_DELAY` | 0.5     | Seconds to wait before refiring after a miss         |
| `hoodIncrement` | 0.5     | How much Cross increments hood override each press   |

## How To Use
1. Select **Teleop** from the TeleOp menu under group **MAIN**
2. During `init_loop` press **DPad Up** for Blue or **DPad Down** for Red
3. Place robot flush against the starting corner wall and press **PLAY**
4. Robot auto-scans spindexer slots — if pre-loaded it enters shoot mode automatically
5. Press **Triangle** to start intaking — drive under balls until spindexer is full
6. Robot switches to shoot mode automatically when full — flywheel starts spinning up
7. Wait for controller rumble — then press **Square** to fire one ball at a time
8. Or press **Cross** when ready to auto-fire all remaining balls
9. After all balls fired robot returns to intake mode automatically
10. If odometry drifts mid-match drive to your starting corner and press **DPad Up** to reset

## Known Issues / Notes
- `shootVelocity` — set to `-1` to use the lookup table value uncapped.
  Set to any positive number to manually override the table (useful for
  testing a specific velocity without redeploying)
- Hood auto-adjusts every loop tick in shoot mode based on robot position —
  no manual tuning needed unless you press Cross to override
- If color sensor misreads a ball press Triangle again to trigger a
  non-blocking rescan (~0.45s, robot stays fully responsive during it)
- DPad Up resets both odometry position AND turret angle offset —
  always do this from the physical starting corner for best accuracy
- Red alliance starting position is `(8.5, 9)` heading `180°`,
  Blue is `(135.5, 9)` heading `0°`
- `intakeOrder[]` tracks the order balls were loaded so shot sequence
  matches the field pattern (GPP/PGP/PPG) correctly