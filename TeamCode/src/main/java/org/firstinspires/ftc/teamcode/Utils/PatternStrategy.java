package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Vision.RampScanner;
import org.firstinspires.ftc.teamcode.robot.Spindexer.DetectedColor;

/**
 * PatternStrategy
 * IMPORTANT: This is the decision engine for the 3-ball scoring bonus. 
 * It calculates whether the combined inventory of the robot (Spindexer) and 
 * the field (Ramp) is sufficient to complete the current randomization pattern.
 * ────────────────────────────────────────────────────────────────────────────
 *Experimental Notes: (Not Final Code Not even sure it works)
 * ── Scoring logic ────────────────────────────────────────────────────────────
 *   PATTERN match = 2 pts per ball (must have all 3 for bonus)
 *   CLASSIFIED (ramp ball stays) = 3 pts each
 *   OVERFLOW (4th+ ball) = 1 pt each
 * ────────────────────────────────────────────────────────────────────────────
 */
public class PatternStrategy {

    private PatternStrategy() {}   // static-only

    /** The result returned to the caller. */
    public static class Decision {

        /**
         * True if combined spindexer + ramp has enough of each color
         * to fire the complete 3-ball pattern.
         */
        public final boolean canScoreFullPattern;

        /**
         * Ordered shoot sequence (length 3, or shorter if we can only do partial).
         * Index 0 = first ball to fire.
         * e.g. for GPP: [GREEN, PURPLE, PURPLE]
         */
        public final DetectedColor[] shootSequence;

        /**
         * How many GREEN balls we need to intake from the ramp before shooting.
         * (0 means spindexer already has enough green.)
         */
        public final int needGreenFromRamp;

        /**
         * How many PURPLE balls we need to intake from the ramp before shooting.
         */
        public final int needPurpleFromRamp;

        /** Convenience: total balls to pull from ramp before shooting. */
        public int needFromRamp() { return needGreenFromRamp + needPurpleFromRamp; }

        Decision(boolean canScoreFullPattern,
                 DetectedColor[] shootSequence,
                 int needGreenFromRamp,
                 int needPurpleFromRamp) {
            this.canScoreFullPattern = canScoreFullPattern;
            this.shootSequence       = shootSequence;
            this.needGreenFromRamp   = needGreenFromRamp;
            this.needPurpleFromRamp  = needPurpleFromRamp;
        }

        @Override public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("canScore=").append(canScoreFullPattern);
            sb.append(" sequence=[");
            for (int i = 0; i < shootSequence.length; i++) {
                if (i > 0) sb.append(", ");
                sb.append(shootSequence[i]);
            }
            sb.append("] needRamp=G").append(needGreenFromRamp)
                    .append("+P").append(needPurpleFromRamp);
            return sb.toString();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /**
     * Core decision method.
     * IMPORTANT: Compares current state against required pattern to find gaps.
     *
     * @param obeliskPattern  what the AprilTag obelisk says (GPP/PGP/PPG/UNKNOWN)
     * @param spindexerBalls  current contents of your 3-slot spindexer
     * @param ramp            what the Limelight sees on the ramp right now
     * @return                a Decision object with everything you need to act
     */
    public static Decision decide(MatchPattern.Pattern obeliskPattern,
                                  DetectedColor[]      spindexerBalls,
                                  RampScanner.RampContents ramp) {

        // ── 1. Count spindexer inventory ─────────────────────────────────────
        int haveGreen  = 0;
        int havePurple = 0;
        for (DetectedColor ball : spindexerBalls) {
            if (ball == DetectedColor.GREEN)  haveGreen++;
            if (ball == DetectedColor.PURPLE) havePurple++;
        }

        // ── 2. Determine what the pattern needs ──────────────────────────────
        int needGreen  = 0;
        int needPurple = 0;
        DetectedColor[] sequence = new DetectedColor[3];

        switch (obeliskPattern) {
            case GPP:
                needGreen  = 1;  needPurple = 2;
                sequence   = new DetectedColor[]{ DetectedColor.GREEN,
                        DetectedColor.PURPLE,
                        DetectedColor.PURPLE };
                break;
            case PGP:
                needGreen  = 1;  needPurple = 2;
                sequence   = new DetectedColor[]{ DetectedColor.PURPLE,
                        DetectedColor.GREEN,
                        DetectedColor.PURPLE };
                break;
            case PPG:
                needGreen  = 1;  needPurple = 2;
                sequence   = new DetectedColor[]{ DetectedColor.PURPLE,
                        DetectedColor.PURPLE,
                        DetectedColor.GREEN };
                break;
            default:
                // UNKNOWN — no pattern to chase, shoot whatever we have
                return decideUnknownPattern(haveGreen, havePurple);
        }

        // ── 3. How much do we still need after spindexer? ────────────────────
        int stillNeedGreen  = Math.max(0, needGreen  - haveGreen);
        int stillNeedPurple = Math.max(0, needPurple - havePurple);

        // ── 4. Can the ramp cover the gap? ───────────────────────────────────
        boolean rampCoversGreen  = ramp.green  >= stillNeedGreen;
        boolean rampCoversPurple = ramp.purple >= stillNeedPurple;
        boolean canScoreFull     = rampCoversGreen && rampCoversPurple;

        if (canScoreFull) {
            // We CAN fire the full pattern (possibly after pulling from ramp)
            return new Decision(true, sequence, stillNeedGreen, stillNeedPurple);
        }

        // ── 5. Can't score full pattern — fire what we have ──────────────────
        // IMPORTANT: We prioritize pattern balls we ALREADY HAVE in the spindexer
        // to maximize points without intaking more.
        DetectedColor[] partial = buildBestPartial(obeliskPattern, haveGreen, havePurple);
        return new Decision(false, partial, 0, 0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    /**
     * When pattern is UNKNOWN, just shoot what we have in score-maximising order.
     */
    private static Decision decideUnknownPattern(int haveGreen, int havePurple) {
        int total = haveGreen + havePurple;
        DetectedColor[] seq = new DetectedColor[Math.min(total, 3)];
        int idx = 0;
        for (int i = 0; i < haveGreen  && idx < seq.length; i++) seq[idx++] = DetectedColor.GREEN;
        for (int i = 0; i < havePurple && idx < seq.length; i++) seq[idx++] = DetectedColor.PURPLE;
        return new Decision(false, seq, 0, 0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    /**
     * Build the best partial sequence from what's in the spindexer.
     * IMPORTANT: Ensures balls that match the pattern are fired first for 2pt scoring.
     */
    private static DetectedColor[] buildBestPartial(MatchPattern.Pattern pattern,
                                                    int haveGreen,
                                                    int havePurple) {
        // Figure out the full desired sequence
        DetectedColor[] desired;
        switch (pattern) {
            case GPP: desired = new DetectedColor[]{ DetectedColor.GREEN,  DetectedColor.PURPLE, DetectedColor.PURPLE }; break;
            case PGP: desired = new DetectedColor[]{ DetectedColor.PURPLE, DetectedColor.GREEN,  DetectedColor.PURPLE }; break;
            case PPG: desired = new DetectedColor[]{ DetectedColor.PURPLE, DetectedColor.PURPLE, DetectedColor.GREEN  }; break;
            default:  desired = new DetectedColor[]{ DetectedColor.GREEN,  DetectedColor.GREEN,  DetectedColor.GREEN  }; break;
        }

        // Walk through desired order and include only what we actually have
        int remG = haveGreen, remP = havePurple;
        java.util.List<DetectedColor> result = new java.util.ArrayList<>();

        for (DetectedColor want : desired) {
            if (want == DetectedColor.GREEN  && remG  > 0) { result.add(DetectedColor.GREEN);  remG--;  }
            if (want == DetectedColor.PURPLE && remP  > 0) { result.add(DetectedColor.PURPLE); remP--;  }
        }

        return result.toArray(new DetectedColor[0]);
    }
}
