package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

@TeleOp(name = "TestLimelight")
public class TestLimelight extends OpMode {

    @Override
    public void init() {
        Drivetrain.getInstance().init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Distance From Red Tag",  Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Angle From Blue Tag",    Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Angle From Red Tag",     Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP" :
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP" :
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG" :
                                        "Can't be found");
        telemetry.update();
    }

    @Override
    public void stop() {
        Limelight.INSTANCE.stop();
    }
}