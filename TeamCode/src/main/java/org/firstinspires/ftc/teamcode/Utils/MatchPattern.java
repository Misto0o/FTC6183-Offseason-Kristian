package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Vision.Limelight;

public class MatchPattern {

    public enum Pattern { GPP, PGP, PPG, UNKNOWN }

    private static Pattern detectedPattern = Pattern.UNKNOWN;
    private static boolean locked = false;

    /**
     * Call every loop tick until isLocked() returns true.
     * Keeps scanning until pattern is found, then locks and shuts Limelight off.
     */
    public static void tryDetect() {
        if (locked) return; // already found, do nothing
        int raw = Limelight.INSTANCE.patternFromObelisk();
        if (raw == Limelight.GPP_PATTERN_ID) {
            detectedPattern = Pattern.GPP;
            locked = true;
            Limelight.INSTANCE.stop(); // shut off only after confirmed lock
        } else if (raw == Limelight.PGP_PATTERN_ID) {
            detectedPattern = Pattern.PGP;
            locked = true;
            Limelight.INSTANCE.stop();
        } else if (raw == Limelight.PPG_PATTERN_ID) {
            detectedPattern = Pattern.PPG;
            locked = true;
            Limelight.INSTANCE.stop();
        }
        // if nothing found yet — keep scanning, do NOT stop
    }

    public static Pattern getPattern() { return detectedPattern; }
    public static boolean isLocked()   { return locked; }

    /** Call in init() to reset between matches. */
    public static void reset() { detectedPattern = Pattern.UNKNOWN; locked = false; }
}