package org.firstinspires.ftc.teamcode.Auto.PairedAuto.bttdb;

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
@Autonomous(name = "Blue Far Auto (Path Test)", group = "Blue")
public class BlueFarAuto extends LinearOpMode {

    // ── Paths ─────────────────────────────────────────────────────────────────
    // NOTE: Path-only test auto — no subsystems, no shooting.
    // Cheick's original used Robot but never called any shoot/intake methods.
    private Path pickUpLastRow, returnOne;
    private Path pickUpLoadingZoneOne, pickUpLoadingZoneTwo, returnTwo;

    // ── Poses ─────────────────────────────────────────────────────────────────
    private final Pose startPose        = new Pose(56, 8, Math.toRadians(90));
    private final Pose shootPose        = new Pose(72, 72);
    private final Pose lastRowPose      = new Pose(14.122, 35.816);
    private final Pose loadingZonePose  = new Pose(5, 8);
    private final Pose loadingZonePose2 = new Pose(5.551, 20.265);
    private final Pose endPose          = new Pose(72.020, 23.327);

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        buildPaths();

        telemetry.addLine("Ready — Blue Far Auto (Path Test)");
        telemetry.addLine("No subsystems — path geometry test only");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        follower.setStartingPose(startPose);

        followPath(follower, pickUpLastRow);
        followPath(follower, returnOne);
        followPath(follower, pickUpLoadingZoneOne);
        followPath(follower, returnTwo);

        // Additional paths available to uncomment as verified:
        // followPath(follower, pickUpLoadingZoneTwo);
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
        // Curve from start into last row of balls
        pickUpLastRow = new Path(new BezierCurve(
                startPose,
                new Pose(51.245, 40.959),
                lastRowPose
        ));
        pickUpLastRow.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        // Return from shoot pose back to start
        returnOne = new Path(new BezierLine(
                shootPose,
                startPose
        ));
        returnOne.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90));

        // Drive to loading zone 1
        pickUpLoadingZoneOne = new Path(new BezierLine(
                startPose,
                loadingZonePose
        ));
        pickUpLoadingZoneOne.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        // Drive to loading zone 2 via curve
        pickUpLoadingZoneTwo = new Path(new BezierCurve(
                startPose,
                new Pose(31.276, 20.316),
                loadingZonePose2
        ));
        pickUpLoadingZoneTwo.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

        // Return from loading zone 2 to end pose
        // NOTE: Cheick's original overwrote returnTwo with this — loading zone 1 return is lost.
        // Keeping that behavior here to match original intent.
        returnTwo = new Path(new BezierLine(
                loadingZonePose2,
                endPose
        ));
        returnTwo.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90));
    }
}