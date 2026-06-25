package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Pinpoint Odometry Subsystem (Final code)
 * Pure odometry using GoBilda Pinpoint driver.
 * This is a pure dead-reckoning system backed by encoder data.
 */
@Config
public class Pinpoint {

    public static final Pinpoint INSTANCE = new Pinpoint();
    private Pinpoint() {}

    private GoBildaPinpointDriver pinpoint;

    // ─────────────────────────────────────────────────────────────────────────
    // Initialization
    // ─────────────────────────────────────────────────────────────────────────
    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set the physical offsets from the pinpoint sensor to the robot's center
        pinpoint.setOffsets(4.5, -7.125, DistanceUnit.INCH);

        // Use GoBilda 4-bar odometry pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Tune these based on your encoder orientation
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Position Setters
    // ─────────────────────────────────────────────────────────────────────────
    /**
     * Manually set the robot's position on the field.
     * Use this at the start() of your opmode to anchor your starting pose.
     * IMPORTANT: Only call this once at match start, not during the match.
     */
    public void updatePosition(Pose2D position) {
        if (pinpoint != null) {
            pinpoint.setPosition(position);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Position Getters
    // ─────────────────────────────────────────────────────────────────────────
    /**
     * @return Current X position in inches (field frame)
     */
    public double getPosX() {
        if (pinpoint == null) return 0;
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    /**
     * @return Current Y position in inches (field frame)
     */
    public double getPosY() {
        if (pinpoint == null) return 0;
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    /**
     * @return Current heading in degrees [0, 360)
     * IMU provides absolute heading from field reference frame.
     */
    public double getHeading() {
        if (pinpoint == null) return 0;
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    /**
     * Corrects X/Y position drift using Limelight distance to a known goal.
     * Heading is LEFT COMPLETELY UNTOUCHED — only position is updated.
     */
    public void relocalizePositionFromTag(double correctedX, double correctedY) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                correctedX,
                correctedY,
                AngleUnit.DEGREES,
                getHeading()   // keep current heading exactly as-is
        ));
    }

    /**
     * Corrects X/Y AND heading using a vision-based pose.
     * Use sparingly — heading corrections should be gentle since Pinpoint's IMU
     * is usually more reliable frame-to-frame than a single vision snapshot.
     */
    public void relocalizeFull(double correctedX, double correctedY, double correctedHeadingDeg) {
        if (pinpoint == null) return;
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                correctedX,
                correctedY,
                AngleUnit.DEGREES,
                correctedHeadingDeg
        ));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic Update (Call once per loop)
    // ─────────────────────────────────────────────────────────────────────────
    /**
     * IMPORTANT: Call this every loop iteration to update the odometry state.
     * This reads the encoders and IMU, updating internal position tracking.
     */
    public void periodic() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }
}