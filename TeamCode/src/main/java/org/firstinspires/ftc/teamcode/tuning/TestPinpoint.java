package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;

@TeleOp(name = "TestPinpoint", group = "Tuning")
public class TestPinpoint extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pinpoint pinpoint = Pinpoint.INSTANCE;
        Drivetrain drivetrain = Drivetrain.getInstance();

        pinpoint.init(hardwareMap);
        drivetrain.init(hardwareMap);

        telemetry.addLine("Pinpoint ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.periodic();

            // Basic mecanum drive so you can move the robot around
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.1; // strafing correction, same as teleop
            double rx =  gamepad1.right_stick_x;
            drivetrain.drive(y, x, rx);

            // Manual reset — press A to zero position and IMU
            if (gamepad1.a) {
                pinpoint.resetPosAndIMU(); // expose this or call through INSTANCE
            }

            double posX    = pinpoint.getPosX();
            double posY    = pinpoint.getPosY();
            double heading = pinpoint.getHeading();

            telemetry.addLine("=== Pinpoint ===");
            telemetry.addData("X (in)",    "%.2f", posX);
            telemetry.addData("Y (in)",    "%.2f", posY);
            telemetry.addData("Heading",   "%.2f°", heading);
            telemetry.addLine("");
            telemetry.addLine("=== Sanity Checks ===");
            telemetry.addData("Driving forward → Y should increase", "");
            telemetry.addData("Strafing right   → X should increase", "");
            telemetry.addData("CCW rotation     → heading increases", "");
            telemetry.addLine("");
            telemetry.addLine("Controls: Drive normally | A = reset pose");
            telemetry.update();
        }
    }
}