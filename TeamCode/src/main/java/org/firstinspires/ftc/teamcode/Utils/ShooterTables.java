package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Utils.Interpolator;

/**
 * ShooterTables Utility
 * IMPORTANT: This class contains the hard-coded calibration data for the shooter.
 * These points are collected using DataCollection.java and define how the robot
 * should behave at various field coordinates.
 *
 * Changes from previous version:
 * - Hood values bumped up across the board (~+0.05 mid-range, +0.08 far zone)
 *   to compensate for shots falling short.
 * - Fixed typo: Blue (144-53, 17) was 150 ticks, corrected to 1350.
 * - Added additional far-zone shooter + hood points (y <= 25) for better
 *   coverage when shooting from the wall / near side.
 */
public class ShooterTables {

    /**
     * IMPORTANT: Maps field (X, Y) to Target Flywheel Velocity (ticks/s) for Blue Alliance.
     */
    public static void loadBlueShooter(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(71.3709, 72.2894, 1100);
        i.addPoint(64,      84,       1075);  // +25
        i.addPoint(51,      90,       1025);  // +25
        i.addPoint(33,      116,       925);  // +25
        i.addPoint(74,      90,       1125);  // +25
        i.addPoint(74,      106,      1125);  // +25
        i.addPoint(72,      129,      1125);  // +25
        i.addPoint(63,      100,      1075);  // +25
        i.addPoint(59,      125,      1075);  // +25
        i.addPoint(97,      89,       1225);  // +25
        i.addPoint(114,     111,      1225);  // +25
        i.addPoint(118,     126,      1225);  // +25
        i.addPoint(87,      105,      1225);  // +25
        i.addPoint(87,      130,      1125);  // +25

        // ── Near-wall / far zone (y <= 25) ───────────────────────────────────
        // IMPORTANT: These points cover shots taken close to the near wall.
        // Typo fix: (144-53, 17) was incorrectly 150, now 1350.
        i.addPoint(144-53,  9,        1400);
        i.addPoint(144-60,  9,        1400);
        i.addPoint(144-76,  9,        1400);
        i.addPoint(144-85,  9,        1400);
        i.addPoint(144-95,  9,        1425);  // new — wider left coverage
        i.addPoint(144-105, 9,        1425);  // new

        i.addPoint(144-53,  17,       1350);  // FIXED (was 150)
        i.addPoint(144-60,  17,       1350);
        i.addPoint(144-76,  17,       1350);
        i.addPoint(144-85,  17,       1350);
        i.addPoint(144-95,  17,       1375);  // new
        i.addPoint(144-105, 17,       1375);  // new

        i.addPoint(144-53,  25,       1350);
        i.addPoint(144-60,  25,       1350);
        i.addPoint(144-76,  25,       1350);
        i.addPoint(144-85,  25,       1350);
        i.addPoint(144-95,  25,       1375);  // new
        i.addPoint(144-105, 25,       1375);  // new

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(100,     70,       1250);  // new — far right of field
        i.addPoint(110,     80,       1250);  // new
        i.addPoint(120,     90,       1250);  // new
        i.addPoint(125,     105,      1250);  // new
    }

    /**
     * IMPORTANT: Maps field (X, Y) to Target Hood Position (0-1) for Blue Alliance.
     * Hood bumped up ~0.05 mid-range and ~0.08 far zone to avoid short shots.
     */
    public static void loadBlueHood(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(71.3709, 72.2894, 0.15);  // +0.05
        i.addPoint(64,      84,       0.15);  // +0.05
        i.addPoint(51,      90,       0.15);  // +0.05
        i.addPoint(33,      116,      0.55);  // +0.05
        i.addPoint(74,      90,       0.20);  // +0.05
        i.addPoint(74,      106,      0.20);  // +0.05
        i.addPoint(72,      129,      0.20);  // +0.05
        i.addPoint(63,      100,      0.20);  // +0.05
        i.addPoint(59,      125,      0.20);  // +0.05
        i.addPoint(97,      89,       0.20);  // +0.05
        i.addPoint(114,     111,      0.20);  // +0.05
        i.addPoint(118,     126,      0.20);  // +0.05
        i.addPoint(87,      105,      0.15);  // +0.05
        i.addPoint(87,      130,      0.15);  // +0.05

        // ── Near-wall / far zone (y <= 25) ───────────────────────────────────
        i.addPoint(144-53,  9,        0.20);  // +0.08
        i.addPoint(144-60,  9,        0.20);
        i.addPoint(144-76,  9,        0.20);
        i.addPoint(144-85,  9,        0.20);
        i.addPoint(144-95,  9,        0.22);  // new
        i.addPoint(144-105, 9,        0.22);  // new

        i.addPoint(144-53,  17,       0.23);  // +0.08
        i.addPoint(144-60,  17,       0.23);
        i.addPoint(144-76,  17,       0.23);
        i.addPoint(144-85,  17,       0.23);
        i.addPoint(144-95,  17,       0.25);  // new
        i.addPoint(144-105, 17,       0.25);  // new

        i.addPoint(144-53,  25,       0.23);  // +0.08
        i.addPoint(144-60,  25,       0.23);
        i.addPoint(144-76,  25,       0.23);
        i.addPoint(144-85,  25,       0.23);
        i.addPoint(144-95,  25,       0.25);  // new
        i.addPoint(144-105, 25,       0.25);  // new

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(100,     70,       0.20);  // new
        i.addPoint(110,     80,       0.20);  // new
        i.addPoint(120,     90,       0.20);  // new
        i.addPoint(125,     105,      0.20);  // new
    }

    /**
     * IMPORTANT: Maps field (X, Y) to Target Flywheel Velocity (ticks/s) for Red Alliance.
     */
    public static void loadRedShooter(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(72.6291, 72.2894, 1100);
        i.addPoint(80,      84,       1075);  // +25
        i.addPoint(93,      90,       1025);  // +25
        i.addPoint(111,     116,       925);  // +25
        i.addPoint(70,      90,       1125);  // +25
        i.addPoint(70,      106,      1125);  // +25
        i.addPoint(72,      129,      1125);  // +25
        i.addPoint(81,      100,      1075);  // +25
        i.addPoint(85,      125,      1075);  // +25
        i.addPoint(47,      89,       1225);  // +25
        i.addPoint(30,      111,      1225);  // +25
        i.addPoint(26,      126,      1225);  // +
        i.addPoint(57,      105,      1225) ;  // +25
        i.addPoint(57,      130,      1125);  // +25

        // ── Near-wall / far zone ──────────────────────────────────────────────
        i.addPoint(53,      9,        1400);
        i.addPoint(60,      9,        1400);
        i.addPoint(76,      9,        1400);
        i.addPoint(85,      9,        1400);
        i.addPoint(95,      9,        1425);  // new
        i.addPoint(105,     9,        1425);  // new

        i.addPoint(53,      17,       1350);
        i.addPoint(60,      17,       1350);
        i.addPoint(76,      17,       1350);
        i.addPoint(85,      17,       1350);
        i.addPoint(95,      17,       1375);  // new
        i.addPoint(105,     17,       1375);  // new

        i.addPoint(53,      25,       1350);
        i.addPoint(60,      25,       1350);
        i.addPoint(76,      25,       1350);
        i.addPoint(85,      25,       1350);
        i.addPoint(95,      25,       1375);  // new
        i.addPoint(105,     25,       1375);  // new

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(44,      70,       1250);  // new
        i.addPoint(34,      80,       1250);  // new
        i.addPoint(24,      90,       1250);  // new
        i.addPoint(19,      105,      1250);  // new
    }

    /**
     * IMPORTANT: Maps field (X, Y) to Target Hood Position (0-1) for Red Alliance.
     * Hood bumped up ~0.05 mid-range and ~0.08 far zone to avoid short shots.
     */
    public static void loadRedHood(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(72.6291, 72.2894, 0.15);  // +0.05
        i.addPoint(80,      84,       0.15);
        i.addPoint(93,      90,       0.15);
        i.addPoint(111,     116,      0.55);  // +0.05
        i.addPoint(70,      90,       0.20);
        i.addPoint(70,      106,      0.20);
        i.addPoint(72,      129,      0.20);
        i.addPoint(81,      100,      0.20);
        i.addPoint(85,      125,      0.20);
        i.addPoint(47,      89,       0.20);
        i.addPoint(30,      111,      0.20);
        i.addPoint(26,      126,      0.20);
        i.addPoint(57,      105,      0.15);
        i.addPoint(57,      130,      0.15);

        // ── Near-wall / far zone ──────────────────────────────────────────────
        i.addPoint(53,      9,        0.20);  // +0.08
        i.addPoint(60,      9,        0.20);
        i.addPoint(76,      9,        0.20);
        i.addPoint(85,      9,        0.20);
        i.addPoint(95,      9,        0.22);  // new
        i.addPoint(105,     9,        0.22);  // new

        i.addPoint(53,      17,       0.23);  // +0.08
        i.addPoint(60,      17,       0.23);
        i.addPoint(76,      17,       0.23);
        i.addPoint(85,      17,       0.23);
        i.addPoint(95,      17,       0.25);  // new
        i.addPoint(105,     17,       0.25);  // new

        i.addPoint(53,      25,       0.23);  // +0.08
        i.addPoint(60,      25,       0.23);
        i.addPoint(76,      25,       0.23);
        i.addPoint(85,      25,       0.23);
        i.addPoint(95,      25,       0.25);  // new
        i.addPoint(105,     25,       0.25);  // new

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(44,      70,       0.20);  // new
        i.addPoint(34,      80,       0.20);  // new
        i.addPoint(24,      90,       0.20);  // new
        i.addPoint(19,      105,      0.20);  // new
    }
}