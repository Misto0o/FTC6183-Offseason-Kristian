package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;

@TeleOp(name = "Drivetrain Test", group = "Tuning")
public class driveTrainTest extends OpMode {

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        telemetry.addLine("Joystick = full mecanum drive via Drivetrain.java");
        telemetry.addLine("DPad Up    = Front Left only");
        telemetry.addLine("DPad Down  = Back Left only");
        telemetry.addLine("DPad Left  = Front Right only");
        telemetry.addLine("DPad Right = Back Right only");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            // DPad: bypass Drivetrain and spin one motor at a time raw
            Drivetrain.getInstance().drive(
                    gamepad1.dpad_up    ? 0.5 : gamepad1.dpad_down  ? -0.5 : 0,
                    gamepad1.dpad_left  ? 0.5 : gamepad1.dpad_right ? -0.5 : 0,
                    0
            );
        } else {
            Drivetrain.getInstance().drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
            );
        }

        telemetry.addData("Y (fwd/back)", -gamepad1.left_stick_y);
        telemetry.addData("X (strafe)",    gamepad1.left_stick_x);
        telemetry.addData("RX (turn)",     gamepad1.right_stick_x);
        telemetry.update();
    }
}