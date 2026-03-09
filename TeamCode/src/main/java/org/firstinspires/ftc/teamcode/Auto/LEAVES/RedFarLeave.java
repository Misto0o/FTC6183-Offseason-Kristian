package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Red Far Leave", group = "Red")
public class RedFarLeave extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        Path farLeave = new Path(new BezierCurve(
                new Pose(85.224, 9),
                new Pose(85.388, 60)
        ));
        farLeave.setConstantHeadingInterpolation(Math.toRadians(90));

        telemetry.addLine("Ready — Red Far Leave");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(new Pose(85.224, 9, Math.toRadians(90)));
        follower.followPath(farLeave, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}