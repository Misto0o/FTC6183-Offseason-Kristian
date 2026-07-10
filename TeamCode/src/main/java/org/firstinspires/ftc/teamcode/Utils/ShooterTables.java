package org.firstinspires.ftc.teamcode.Utils;

/**
 * ShooterTables V2 — High Hood Experiment
 * Hood pushed toward 1.0 based on field-testing showing better accuracy at higher hood positions.
 * RPM kept the same as V1 — adjust only if shots fall short.
 */
public class ShooterTables {

    public static void loadBlueShooter(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(71.3709, 72.2894, 1100);
        i.addPoint(64,      84,       1075);
        i.addPoint(51,      90,       1025);
        i.addPoint(33,      116,       925);
        i.addPoint(74,      90,       1125);
        i.addPoint(74,      106,      1125);
        i.addPoint(72,      129,      1125);
        i.addPoint(63,      100,      1075);
        i.addPoint(59,      125,      1075);
        i.addPoint(97,      89,       1225);
        i.addPoint(114,     111,      1225);
        i.addPoint(118,     126,      1225);
        i.addPoint(87,      105,      1225);
        i.addPoint(87,      130,      1125);

        // ── Near-wall / far zone (y <= 25) ───────────────────────────────────
        i.addPoint(144-53,  9,        1400);
        i.addPoint(144-60,  9,        1400);
        i.addPoint(144-76,  9,        1400);
        i.addPoint(144-85,  9,        1400);
        i.addPoint(144-95,  9,        1425);
        i.addPoint(144-105, 9,        1425);

        i.addPoint(144-53,  17,       1350);
        i.addPoint(144-60,  17,       1350);
        i.addPoint(144-76,  17,       1350);
        i.addPoint(144-85,  17,       1350);
        i.addPoint(144-95,  17,       1375);
        i.addPoint(144-105, 17,       1375);

        i.addPoint(144-53,  25,       1350);
        i.addPoint(144-60,  25,       1350);
        i.addPoint(144-76,  25,       1350);
        i.addPoint(144-85,  25,       1350);
        i.addPoint(144-95,  25,       1375);
        i.addPoint(144-105, 25,       1375);

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(100,     70,       1250);
        i.addPoint(110,     80,       1250);
        i.addPoint(120,     90,       1250);
        i.addPoint(125,     105,      1250);
    }

    public static void loadBlueHood(Interpolator i) {
        // ── Mid-field — pushed toward 1.0 ────────────────────────────────────
        i.addPoint(71.3709, 72.2894, 0.85);
        i.addPoint(64,      84,       0.85);
        i.addPoint(51,      90,       0.85);
        i.addPoint(33,      116,      0.80);
        i.addPoint(74,      90,       0.85);
        i.addPoint(74,      106,      0.85);
        i.addPoint(72,      129,      0.80);
        i.addPoint(63,      100,      0.85);
        i.addPoint(59,      125,      0.80);
        i.addPoint(97,      89,       0.85);
        i.addPoint(114,     111,      0.85);
        i.addPoint(118,     126,      0.85);
        i.addPoint(87,      105,      0.85);
        i.addPoint(87,      130,      0.80);

        // ── Near-wall / far zone (y <= 25) — sniper zone ─────────────────────
        i.addPoint(144-53,  9,        1.0);
        i.addPoint(144-60,  9,        1.0);
        i.addPoint(144-76,  9,        1.0);
        i.addPoint(144-85,  9,        1.0);
        i.addPoint(144-95,  9,        1.0);
        i.addPoint(144-105, 9,        1.0);

        i.addPoint(144-53,  17,       1.0);
        i.addPoint(144-60,  17,       1.0);
        i.addPoint(144-76,  17,       1.0);
        i.addPoint(144-85,  17,       1.0);
        i.addPoint(144-95,  17,       1.0);
        i.addPoint(144-105, 17,       1.0);

        i.addPoint(144-53,  25,       0.95);
        i.addPoint(144-60,  25,       0.95);
        i.addPoint(144-76,  25,       0.95);
        i.addPoint(144-85,  25,       0.95);
        i.addPoint(144-95,  25,       0.95);
        i.addPoint(144-105, 25,       0.95);

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(100,     70,       0.85);
        i.addPoint(110,     80,       0.85);
        i.addPoint(120,     90,       0.85);
        i.addPoint(125,     105,      0.85);
    }

    public static void loadRedShooter(Interpolator i) {
        // ── Mid-field ─────────────────────────────────────────────────────────
        i.addPoint(72.6291, 72.2894, 1100);
        i.addPoint(80,      84,       1075);
        i.addPoint(93,      90,       1025);
        i.addPoint(111,     116,       925);
        i.addPoint(70,      90,       1125);
        i.addPoint(70,      106,      1125);
        i.addPoint(72,      129,      1125);
        i.addPoint(81,      100,      1075);
        i.addPoint(85,      125,      1075);
        i.addPoint(47,      89,       1225);
        i.addPoint(30,      111,      1225);
        i.addPoint(26,      126,      1225);
        i.addPoint(57,      105,      1225);
        i.addPoint(57,      130,      1125);

        // ── Near-wall / far zone ──────────────────────────────────────────────
        i.addPoint(53,      9,        1400);
        i.addPoint(60,      9,        1400);
        i.addPoint(76,      9,        1400);
        i.addPoint(85,      9,        1400);
        i.addPoint(95,      9,        1425);
        i.addPoint(105,     9,        1425);

        i.addPoint(53,      17,       1350);
        i.addPoint(60,      17,       1350);
        i.addPoint(76,      17,       1350);
        i.addPoint(85,      17,       1350);
        i.addPoint(95,      17,       1375);
        i.addPoint(105,     17,       1375);

        i.addPoint(53,      25,       1350);
        i.addPoint(60,      25,       1350);
        i.addPoint(76,      25,       1350);
        i.addPoint(85,      25,       1350);
        i.addPoint(95,      25,       1375);
        i.addPoint(105,     25,       1375);

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(44,      70,       1250);
        i.addPoint(34,      80,       1250);
        i.addPoint(24,      90,       1250);
        i.addPoint(19,      105,      1250);
    }

    public static void loadRedHood(Interpolator i) {
        // ── Mid-field — pushed toward 1.0 ────────────────────────────────────
        i.addPoint(72.6291, 72.2894, 0.85);
        i.addPoint(80,      84,       0.85);
        i.addPoint(93,      90,       0.85);
        i.addPoint(111,     116,      0.80);
        i.addPoint(70,      90,       0.85);
        i.addPoint(70,      106,      0.85);
        i.addPoint(72,      129,      0.80);
        i.addPoint(81,      100,      0.85);
        i.addPoint(85,      125,      0.80);
        i.addPoint(47,      89,       0.85);
        i.addPoint(30,      111,      0.85);
        i.addPoint(26,      126,      0.85);
        i.addPoint(57,      105,      0.85);
        i.addPoint(57,      130,      0.80);

        // ── Near-wall / far zone — sniper zone ────────────────────────────────
        i.addPoint(53,      9,        1.0);
        i.addPoint(60,      9,        1.0);
        i.addPoint(76,      9,        1.0);
        i.addPoint(85,      9,        1.0);
        i.addPoint(95,      9,        1.0);
        i.addPoint(105,     9,        1.0);

        i.addPoint(53,      17,       1.0);
        i.addPoint(60,      17,       1.0);
        i.addPoint(76,      17,       1.0);
        i.addPoint(85,      17,       1.0);
        i.addPoint(95,      17,       1.0);
        i.addPoint(105,     17,       1.0);

        i.addPoint(53,      25,       0.95);
        i.addPoint(60,      25,       0.95);
        i.addPoint(76,      25,       0.95);
        i.addPoint(85,      25,       0.95);
        i.addPoint(95,      25,       0.95);
        i.addPoint(105,     25,       0.95);

        // ── Extra far mid-angle coverage ──────────────────────────────────────
        i.addPoint(44,      70,       0.85);
        i.addPoint(34,      80,       0.85);
        i.addPoint(24,      90,       0.85);
        i.addPoint(19,      105,      0.85);
    }
}