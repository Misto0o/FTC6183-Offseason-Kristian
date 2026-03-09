package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;

@Autonomous(name = "Blue Far Leave", group = "Blue")
public class BlueFarLeave extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);

        Path farLeave = new Path(new BezierCurve(
                new Pose(58.959, 9),
                new Pose(58.755, 20)
        ));
        farLeave.setConstantHeadingInterpolation(Math.toRadians(90));

        telemetry.addLine("Ready — Blue Far Leave");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(new Pose(58.959, 9, Math.toRadians(90)));
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 58.959, 9, AngleUnit.DEGREES, 90)
        );

        follower.followPath(farLeave, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pinpoint.INSTANCE.periodic();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}