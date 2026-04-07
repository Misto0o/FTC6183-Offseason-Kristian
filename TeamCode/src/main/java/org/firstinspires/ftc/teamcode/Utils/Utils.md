# Utils Package
**Package:** `org.firstinspires.ftc.teamcode.Utils`
**Last Updated:** April 2026

## What This Is
Shared utility classes used across teleop, autonomous, and tuning opmodes.
These are not opmodes themselves — they are helper classes that other code
depends on. Do not modify these unless you understand what depends on them.

---

## Alliance.java
Simple enum used to select alliance color. Passed into subsystems that
need to know which goal to target or which color pattern to detect.

**Values**

| Value | Meaning |
|-------|---------|
| `BLUE` | Blue alliance |
| `RED` | Red alliance |

---

## Interpolator.java
2D lookup table with inverse distance weighting (IDW) interpolation.
Takes a set of scattered `(x, y, value)` data points and interpolates
a value for any `(x, y)` query — no grid required.

Used by `ShooterTables.java` to look up flywheel velocity and hood
position based on robot coordinates on the field.

**Key Methods**

| Method | What it does |
|--------|--------------|
| `addPoint(x, y, value)` | Adds a data point to the table |
| `get(x, y)` | Returns interpolated value at `(x, y)` |
| `size()` | Returns number of data points loaded |

**How It Works**
- Each query point is weighted by inverse squared distance to every data point
- Exact coordinate matches return instantly without interpolation
- Throws `IllegalStateException` if called on an empty table

---

## MatchPattern.java
Static class that detects and locks the game's motif pattern from the
obelisk via Limelight. Call `tryDetect()` every loop tick during init
until `isLocked()` returns true, then read `getPattern()` for the result.
Automatically shuts off the Limelight once the pattern is confirmed.

**Pattern Values**

| Value | Meaning |
|-------|---------|
| `GPP` | Green, Purple, Purple |
| `PGP` | Purple, Green, Purple |
| `PPG` | Purple, Purple, Green |
| `UNKNOWN` | Not yet detected |

**Key Methods**

| Method | What it does |
|--------|--------------|
| `tryDetect()` | Scans for pattern — call every loop tick until locked |
| `getPattern()` | Returns the detected `Pattern` enum value |
| `isLocked()` | Returns true once pattern is confirmed |
| `reset()` | Resets state — call in `init()` between matches |

**Known Issues / Notes**
- Once locked, `tryDetect()` does nothing — safe to keep calling
- Limelight is shut off automatically after lock to save resources
- Always call `reset()` in `init()` or stale data from last match carries over

---

## SensorColor.java
Tuning opmode for calibrating the left and right color sensors. Displays
live HSV readings and compares them against the hue ranges defined in
`Spindexer.java`. Use this to find correct hue bounds for purple and
green ball detection.

**Hardware Used:** `"leftColorSensor"`, `"rightColorSensor"`

**Controls**

| Button | Action |
|--------|--------|
| Cross | Increase left sensor gain |
| Triangle | Decrease left sensor gain |
| Circle | Increase right sensor gain |
| Square | Decrease right sensor gain |

**How To Use**

1. Run the opmode
2. Hold a purple ball under the left sensor — note the hue reading
3. Hold a green ball under the left sensor — note the hue reading
4. Repeat for the right sensor
5. Update the hue range constants in `Spindexer.java` to match
6. Confirmed detection shows under `── DETECTION ──` on the dashboard

**Known Issues / Notes**
- Current hue ranges from `Spindexer.java` are displayed live — no need
  to cross-reference the file manually while tuning
- Gain defaults to 2 — increase if readings are too low in dim lighting

## ShooterTables
Static loader that populates Interpolator instances for flywheel velocity
and hood position for both alliances. Data points sourced from field testing
via DataCollection.java.

**How to Use**
- `loadBlueShooter()`, `loadBlueHood()`, `loadRedShooter()`, `loadRedHood()`
  are called in `init()`

**Notes**
See the ShooterTable spreadsheet for how data is structured and how to
add new points: [ShooterTable Spreadsheet](https://docs.google.com/spreadsheets/d/1oTGg8vLRNqRh52t9FsZwadkLCS6sTmQx/edit?usp=sharing&ouid=110208191089846695150&rtpof=true&sd=true)

Red and Blue tables are mirrored across the 144" field centerline.
loadBlueShooter / loadBlueHood / loadRedShooter / loadRedHood