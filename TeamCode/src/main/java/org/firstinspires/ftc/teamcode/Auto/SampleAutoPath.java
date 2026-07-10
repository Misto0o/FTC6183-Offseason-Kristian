package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Sample Auto Path", group = "Autonomous")
public class SampleAutoPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_TOPROW,
        DRIVE_TOPROW_COLLECT,
        DRIVE_TOPROW_SHOOT,
        DRIVE_SHOOTPOS_MIDDLEROW,
        DRIVE_MIDDLEROW_COLLECT,
        DRIVE_MIDDLEROW_SHOOT,
        DRIVE_SHOOTPOS_BOTTOMROW,
        DRIVE_BOTTOMROW_COLLECT,
        DRIVE_BOTTOMROW_SHOOT,
        SHOOT_PRELOAD,
        STATIONARY_DONE
    }

    private PathState pathState;

    // Robot Poses matching your generator coordinates
    private final Pose startPose = new Pose(123.42857142857146, 122.57851239669421, Math.toRadians(48));
    private final Pose topRowStart = new Pose(85.17591499409687, 84.6658795749705, Math.toRadians(0));
    private final Pose topRowEnd = new Pose(121.21841794569075, 83.98583234946874, Math.toRadians(0));
    private final Pose shootPose = new Pose(85.17591499409687, 84.6658795749705, Math.toRadians(270));
    private final Pose middleRowStart = new Pose(85.17591499409687, 58.96576151121609, Math.toRadians(0));
    private final Pose middleRowEnd = new Pose(121.21841794569075, 58.96576151121609, Math.toRadians(0));
    private final Pose bottomRowStart = new Pose(85.17591499409687, 35.532467532467564, Math.toRadians(0));
    private final Pose bottomRowEnd = new Pose(121.21841794569075, 35.532467532467564, Math.toRadians(0));

    // PathChains
    private PathChain startPosTopRow;
    private PathChain collectTopRow;
    private PathChain topRowShootPos;
    private PathChain shootPosMiddleRow;
    private PathChain collectMiddleRow;
    private PathChain middleRowShootPos;
    private PathChain shootPosBottomRow;
    private PathChain collectBottomRow;
    private PathChain bottomRowShootPos;

    public void buildPaths() {
        startPosTopRow = follower.pathBuilder()
                .addPath(new BezierLine(startPose, topRowStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), topRowStart.getHeading())
                .build();

        collectTopRow = follower.pathBuilder()
                .addPath(new BezierLine(topRowStart, topRowEnd))
                .setLinearHeadingInterpolation(topRowStart.getHeading(), topRowEnd.getHeading())
                .build();

        topRowShootPos = follower.pathBuilder()
                .addPath(new BezierLine(topRowEnd, shootPose))
                .setLinearHeadingInterpolation(topRowEnd.getHeading(), shootPose.getHeading())
                .build();

        shootPosMiddleRow = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, middleRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), middleRowStart.getHeading())
                .build();

        collectMiddleRow = follower.pathBuilder()
                .addPath(new BezierLine(middleRowStart, middleRowEnd))
                .setLinearHeadingInterpolation(middleRowStart.getHeading(), middleRowEnd.getHeading())
                .build();

        middleRowShootPos = follower.pathBuilder()
                .addPath(new BezierLine(middleRowEnd, shootPose))
                .setLinearHeadingInterpolation(middleRowEnd.getHeading(), shootPose.getHeading())
                .build();

        shootPosBottomRow = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, bottomRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), bottomRowStart.getHeading())
                .build();

        collectBottomRow = follower.pathBuilder()
                .addPath(new BezierLine(bottomRowStart, bottomRowEnd))
                .setLinearHeadingInterpolation(bottomRowStart.getHeading(), bottomRowEnd.getHeading())
                .build();

        bottomRowShootPos = follower.pathBuilder()
                .addPath(new BezierLine(bottomRowEnd, shootPose))
                .setLinearHeadingInterpolation(bottomRowEnd.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_TOPROW:
                if (!follower.isBusy()) {
                    follower.followPath(startPosTopRow, true);
                    setPathState(PathState.DRIVE_TOPROW_COLLECT);
                }
                break;

            case DRIVE_TOPROW_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(collectTopRow, true);
                    setPathState(PathState.DRIVE_TOPROW_SHOOT);
                }
                break;

            case DRIVE_TOPROW_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(topRowShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // TODO: Trigger physical shooter mechanisms here
                    // Once mechanism completes (or after a timer threshold), move state forward:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        follower.followPath(shootPosMiddleRow, true);
                        setPathState(PathState.DRIVE_MIDDLEROW_COLLECT);
                    }
                }
                break;

            case DRIVE_MIDDLEROW_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(collectMiddleRow, true);
                    setPathState(PathState.DRIVE_MIDDLEROW_SHOOT);
                }
                break;

            case DRIVE_MIDDLEROW_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(middleRowShootPos, true);
                    // Repurpose or create a second shoot state if needed, routing to bottom row here:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        follower.followPath(shootPosBottomRow, true);
                        setPathState(PathState.DRIVE_BOTTOMROW_COLLECT);
                    }
                }
                break;

            case DRIVE_BOTTOMROW_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(collectBottomRow, true);
                    setPathState(PathState.DRIVE_BOTTOMROW_SHOOT);
                }
                break;

            case DRIVE_BOTTOMROW_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(bottomRowShootPos, true);
                    setPathState(PathState.STATIONARY_DONE);
                }
                break;

            case STATIONARY_DONE:
                telemetry.addLine("Autonomous Route Finished Successfully.");
                break;

            default:
                telemetry.addLine("No path state selected.");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(PathState.DRIVE_STARTPOS_TOPROW);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
    }
}