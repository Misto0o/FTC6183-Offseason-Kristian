package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {

    public static final Pinpoint INSTANCE = new Pinpoint();
    private Pinpoint() {}

    private GoBildaPinpointDriver pinpoint;

    // Initialize the hardware
    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(4.5, -7.125, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }

    // Update the robot's position
    public void updatePosition(Pose2D position) {
        pinpoint.setPosition(position);
    }

    // Getters
    public double getPosX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getPosY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void resetPosAndIMU() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    // Periodic update
    public void periodic() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }
}