package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Simple Auto", group = "Auto")
public class SimpleAuto extends LinearOpMode {

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double PATH_TIMEOUT = 5.0;

    private PathChain path1, path2, path3, path4, path5,
            path6, path7, path8, path9, path10;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, 0));

        buildPaths();

        telemetry.addLine("Ready — press PLAY");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) return;

        follower.startTeleopDrive();

        followPath(path1);
        followPath(path2);
        followPath(path3);
        followPath(path4);
        followPath(path5);
        followPath(path6);
        followPath(path7);
        followPath(path8);
        followPath(path9);
        followPath(path10);

        telemetry.addLine("Auto complete!");
        telemetry.update();
    }

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(47.620, 35.627)))
                .setTangentHeadingInterpolation()
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(47.620, 35.627), new Pose(23.930, 35.549)))
                .setTangentHeadingInterpolation()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.930, 35.549), new Pose(71.792, 73.981)))
                .setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(71.792, 73.981), new Pose(47.950, 59.744)))
                .setTangentHeadingInterpolation()
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(47.950, 59.744), new Pose(24.364, 59.835)))
                .setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.364, 59.835), new Pose(71.370, 74.349)))
                .setTangentHeadingInterpolation()
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(71.370, 74.349), new Pose(48.366, 84.011)))
                .setTangentHeadingInterpolation()
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.366, 84.011), new Pose(23.632, 83.816)))
                .setTangentHeadingInterpolation()
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.632, 83.816), new Pose(48.213, 84.118)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.213, 84.118), new Pose(71.851, 74.295)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private void followPath(PathChain path) {
        follower.followPath(path, true);
        timer.reset();
        while (opModeIsActive()
                && follower.isBusy()
                && timer.seconds() < PATH_TIMEOUT) {
            follower.update();
            telemetry.addData("Path busy", follower.isBusy());
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
        follower.update();
    }
}