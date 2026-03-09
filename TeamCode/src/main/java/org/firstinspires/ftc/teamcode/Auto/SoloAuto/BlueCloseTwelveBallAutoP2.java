package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Config
@Autonomous(name = "Blue Close Twelve Ball Auto P2 (Path Test)", group = "Blue")
public class BlueCloseTwelveBallAutoP2 extends LinearOpMode {

    // ── Paths ─────────────────────────────────────────────────────────────────
    // NOTE: This is a path-layout test auto — no subsystems, no shooting.
    // Cheick's original had subsystems commented out of addComponents().
    // Use this to verify P2 path geometry on the field before integrating
    // into BlueCloseTwelveBallAuto.
    private Path scorePreload, pickUpSecondRow;
    private Path gateOne, gateTwo, returnTwo;
    private Path path6, pickUpFirstRow, returnOne;
    private Path pickUpThirdRow, end;

    private final Pose startPose = new Pose(24.980, 127.469, Math.toRadians(142));

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        buildPaths();

        telemetry.addLine("Ready — Blue Close Twelve Ball Auto P2 (Path Test)");
        telemetry.addLine("No subsystems — path geometry test only");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(startPose);

        // ── Drive paths in sequence ───────────────────────────────────────────
        followPath(follower, scorePreload);
        followPath(follower, pickUpSecondRow);

        // Remaining paths available to uncomment as each is verified:
        // followPath(follower, gateOne);
        // followPath(follower, gateTwo);
        // followPath(follower, returnTwo);
        // followPath(follower, path6);
        // followPath(follower, pickUpFirstRow);
        // followPath(follower, returnOne);
        // followPath(follower, pickUpThirdRow);
        // followPath(follower, end);
    }

    private void followPath(Follower follower, Path path) {
        follower.followPath(path, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(
                new Pose(24.980, 127.469),
                new Pose(61.531, 84.857)
        ));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        pickUpSecondRow = new Path(new BezierCurve(
                new Pose(61.531, 84.857),
                new Pose(68.908, 56.918),
                new Pose(17.837, 59.898)
        ));
        pickUpSecondRow.setConstantHeadingInterpolation(Math.toRadians(180));

        gateOne = new Path(new BezierLine(
                new Pose(17.837, 59.898),
                new Pose(18.041, 67.816)
        ));
        gateOne.setConstantHeadingInterpolation(Math.toRadians(90));

        gateTwo = new Path(new BezierLine(
                new Pose(18.041, 67.816),
                new Pose(13.755, 67.714)
        ));
        gateTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        returnTwo = new Path(new BezierLine(
                new Pose(13.755, 67.714),
                new Pose(61.531, 72.449)
        ));
        returnTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        path6 = new Path(new BezierLine(
                new Pose(61.531, 72.449),
                new Pose(61.449, 84.918)
        ));
        path6.setConstantHeadingInterpolation(Math.toRadians(90));

        pickUpFirstRow = new Path(new BezierLine(
                new Pose(61.449, 84.918),
                new Pose(17.837, 84.143)
        ));
        pickUpFirstRow.setConstantHeadingInterpolation(Math.toRadians(180));

        returnOne = new Path(new BezierLine(
                new Pose(17.837, 84.143),
                new Pose(61.612, 85.122)
        ));
        returnOne.setConstantHeadingInterpolation(Math.toRadians(180));

        pickUpThirdRow = new Path(new BezierCurve(
                new Pose(61.612, 85.122),
                new Pose(58.592, 30.673),
                new Pose(15.776, 35.633)
        ));
        pickUpThirdRow.setTangentHeadingInterpolation();

        end = new Path(new BezierLine(
                new Pose(15.776, 35.633),
                new Pose(61.490, 85.061)
        ));
        end.setTangentHeadingInterpolation();
    }
}