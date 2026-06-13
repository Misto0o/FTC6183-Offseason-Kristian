package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.robot.Spindexer.DetectedColor;

import java.util.List;

/**
 * RampScanner Utility
 * IMPORTANT: This class uses the Limelight's Neural Detector pipeline to count balls 
 * on the field ramp. This data is fed into the PatternStrategy to optimize scoring.
 */
public class RampScanner {

    // IMPORTANT: These labels must exactly match the labels in your .tflite model.
    public static String CLASS_GREEN  = "g";
    public static String CLASS_PURPLE = "p";

    /**
     * IMPORTANT: Minimum detection area to filter out noise and false positives.
     */
    public static double MIN_AREA = 0.5;

    private RampScanner() {}

    public static class RampContents {
        public final int green;
        public final int purple;

        RampContents(int green, int purple) {
            this.green  = green;
            this.purple = purple;
        }

        public int total()     { return green + purple; }
        public boolean empty() { return total() == 0;   }

        @Override public String toString() {
            return "RampContents{green=" + green + ", purple=" + purple + "}";
        }
    }

    /**
     * IMPORTANT: Performs a single-frame vision scan. 
     * Returns a count of green and purple balls detected.
     */
    public static RampContents scan() {
        LLResult result = Limelight.INSTANCE.getRawResult();
        if (result == null) return new RampContents(0, 0);

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null || detections.isEmpty()) return new RampContents(0, 0);

        int g = 0, p = 0;
        for (LLResultTypes.DetectorResult det : detections) {
            if (det.getTargetArea() < MIN_AREA) continue;
            String label = det.getClassName();
            if      (CLASS_GREEN.equalsIgnoreCase(label))  g++;
            else if (CLASS_PURPLE.equalsIgnoreCase(label)) p++;
        }
        return new RampContents(g, p);
    }

    /**
     * IMPORTANT: Averages multiple frames to prevent "flickering" in the ball count 
     * due to momentary detection drops.
     */
    public static RampContents scanAveraged(int samples, long sleepMs) {
        int totalG = 0, totalP = 0, valid = 0;
        for (int i = 0; i < samples; i++) {
            RampContents snap = scan();
            if (!snap.empty()) { totalG += snap.green; totalP += snap.purple; valid++; }
            if (sleepMs > 0) try { Thread.sleep(sleepMs); } catch (InterruptedException ignored) {}
        }
        if (valid == 0) return new RampContents(0, 0);
        return new RampContents(
                (int) Math.round((double) totalG / valid),
                (int) Math.round((double) totalP / valid)
        );
    }

    public static RampContents fromSpindexer(DetectedColor[] balls) {
        int g = 0, p = 0;
        for (DetectedColor b : balls) {
            if (b == DetectedColor.GREEN)  g++;
            if (b == DetectedColor.PURPLE) p++;
        }
        return new RampContents(g, p);
    }
}
