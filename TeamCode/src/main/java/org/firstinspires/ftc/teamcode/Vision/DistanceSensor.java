package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Distance Sensor Utility
 * IMPORTANT: This sensor is used at the shooter exit to confirm a ball has been fired.
 * NOTE: Currently reported as off the bot due to mechanical changes, but logic remains for future use.
 */
@Config
public class DistanceSensor {
    private Rev2mDistanceSensor sensor;

    // IMPORTANT: Calibration values for ball detection. 
    // If the sensor is re-mounted, these MUST be re-tuned on FTC Dashboard.
    public static double BALL_MIN_CM = 3.0; 
    public static double BALL_MAX_CM = 15.0;

    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    /**
     * @return Distance in CM. Returns 999 as a safe default if the sensor is missing or reading error.
     */
    public double getDistanceCM() {
        if (sensor == null) return 999;
        try {
            double d = sensor.getDistance(DistanceUnit.CM);
            if (Double.isNaN(d) || Double.isInfinite(d)) return 999;
            return d;
        } catch (Exception e) {
            return 999; // Return safe default if sensor throws
        }
    }

    /**
     * IMPORTANT: Used by the Auto-Shoot state machine to confirm a shot.
     */
    public boolean isBallPresent() {
        double d = getDistanceCM();
        return d >= BALL_MIN_CM && d <= BALL_MAX_CM;
    }

    /**
     * @return true if no ball is detected in the exit path.
     */
    public boolean isClear() {
        if (sensor == null) return true;  // Safe default if sensor is missing
        return !isBallPresent();
    }
}
