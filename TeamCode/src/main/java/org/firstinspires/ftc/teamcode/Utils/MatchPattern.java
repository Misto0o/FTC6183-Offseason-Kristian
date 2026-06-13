package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Vision.Limelight;

/**
 * MatchPattern Utility
 * IMPORTANT: This class manages the detection and locking of the randomization pattern.
 * Once a pattern is detected, it "locks" to prevent changing during the match,
 * and shuts down the vision pipeline to save resources.
 */
public class MatchPattern {

    public enum Pattern { GPP, PGP, PPG, UNKNOWN }

    private static Pattern detectedPattern = Pattern.UNKNOWN;
    private static boolean locked = false;

    /**
     * IMPORTANT: Call this during the init_loop of your OpMode.
     * It will continuously poll the Limelight until a valid pattern ID is seen.
     */
    public static void tryDetect() {
        if (locked) return; // Already locked, no need to keep processing.
        
        int raw = Limelight.INSTANCE.patternFromObelisk();
        
        // Match the raw AprilTag ID to our internal Pattern enum.
        if (raw == Limelight.GPP_PATTERN_ID) {
            detectedPattern = Pattern.GPP;
            locked = true;
            Limelight.INSTANCE.stop(); // IMPORTANT: Shuts down vision to save CPU/bandwidth.
        } else if (raw == Limelight.PGP_PATTERN_ID) {
            detectedPattern = Pattern.PGP;
            locked = true;
            Limelight.INSTANCE.stop();
        } else if (raw == Limelight.PPG_PATTERN_ID) {
            detectedPattern = Pattern.PPG;
            locked = true;
            Limelight.INSTANCE.stop();
        }
    }

    public static Pattern getPattern() { return detectedPattern; }
    public static boolean isLocked()   { return locked; }

    /** 
     * IMPORTANT: Must call in init() to clear any state from a previous match. 
     */
    public static void reset() { detectedPattern = Pattern.UNKNOWN; locked = false; }
}
