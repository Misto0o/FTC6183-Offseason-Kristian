package org.firstinspires.ftc.teamcode.Pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing Constants
 * IMPORTANT: This file contains the master tuning values for the robot's movement.
 * Do not change these unless you are performing a full re-tune of the follower.
 */
public class Constants {
    
    // IMPORTANT: Physical and PID constants for the path follower.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(35) // kg
            .forwardZeroPowerAcceleration(-43.18)
            .lateralZeroPowerAcceleration(-64)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.175,0,0.01,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0025,0,0.00001,0.6,0.01))
            .centripetalScaling(0.03);

    // IMPORTANT: Drive motor assignments and directions for the mecanum drivetrain.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(72)
            .yVelocity(49)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE); // Match DT if breaks go back to Reverse

    // IMPORTANT: Localization constants for the Pinpoint computer.
    // Pod offsets are in inches relative to the center of rotation.
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.125)
            .strafePodX(4.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // Constraints that define the speed and acceleration limits of the paths.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 1);

    /**
     * IMPORTANT: Factory method to create a pre-configured Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
