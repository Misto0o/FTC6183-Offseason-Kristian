package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Vision.Limelight;

public class MatchPattern {

    public enum Pattern { GPP, PGP, PPG, UNKNOWN }

    private static Pattern detectedPattern = Pattern.UNKNOWN;
    private static boolean locked = false;

    /** Call this in a loop at the start of AUTO until isLocked() returns true. */
    public static void tryDetect() {
        if (locked) return;
        int raw = Limelight.INSTANCE.patternFromObelisk();
        if (raw == Limelight.GPP_PATTERN_ID) { detectedPattern = Pattern.GPP; locked = true; }
        else if (raw == Limelight.PGP_PATTERN_ID) { detectedPattern = Pattern.PGP; locked = true; }
        else if (raw == Limelight.PPG_PATTERN_ID) { detectedPattern = Pattern.PPG; locked = true; }
    }

    public static Pattern getPattern() { return detectedPattern; }
    public static boolean isLocked()   { return locked; }

    /** Call this in init() so it resets between matches. */
    public static void reset() { detectedPattern = Pattern.UNKNOWN; locked = false; }
}