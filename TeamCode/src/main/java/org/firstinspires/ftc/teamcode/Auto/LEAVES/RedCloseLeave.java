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

@Autonomous(name = "Red Close Leave", group = "Red")
public class RedCloseLeave extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);

        Path closeLeave = new Path(new BezierCurve(
                new Pose(116.265, 130.959),
                new Pose(90.898, 103.531)
        ));
        closeLeave.setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(90));

        telemetry.addLine("Ready — Red Close Leave");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(new Pose(116.265, 130.959, Math.toRadians(38)));
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 116.265, 130.959, AngleUnit.DEGREES, 38)
        );

        follower.followPath(closeLeave, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pinpoint.INSTANCE.periodic();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }
}