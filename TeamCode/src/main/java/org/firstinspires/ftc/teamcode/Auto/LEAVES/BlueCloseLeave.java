package org.firstinspires.ftc.teamcode.Auto.LEAVES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Blue Close Leave", group = "Blue")
public class BlueCloseLeave extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        Path closeLeave = new Path(new BezierCurve(
                new Pose(26.999, 129.490),
                new Pose(54.163, 98.755)
        ));
        closeLeave.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(90));

        telemetry.addLine("Ready — Blue Close Leave");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(new Pose(26.999, 129.490, Math.toRadians(142)));
        follower.followPath(closeLeave, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}