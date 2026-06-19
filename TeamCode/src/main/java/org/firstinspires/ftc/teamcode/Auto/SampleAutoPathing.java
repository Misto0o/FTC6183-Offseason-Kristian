package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

@Autonomous
public class SampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_PICKUPTHREE,
        DRIVE_PICKUPTHREE_SHOOTTHREE,
        SHOOT_THREE,
        DRIVE_SHOOTTHREE_ENDPOS,
        DONE
    }
    PathState pathState;

    // -- Poses --
    private final Pose startPos      = new Pose(20.486, 122.312, Math.toRadians(138));
    private final Pose shootPos      = new Pose(60.162, 83.168,  Math.toRadians(138));
    private final Pose pickupThree   = new Pose(13.954, 83.156,  Math.toRadians(180));
    private final Pose shootThreePos = new Pose(49.000, 101.000, Math.toRadians(138));
    private final Pose endPos        = new Pose(69.364, 118.116, Math.toRadians(138));

    // -- Paths --
    private PathChain driveStartShoot,    // Path 1: start -> shootPos
            driveShootPickup,   // Path 2: shootPos -> pickupThree
            drivePickupShoot,   // Path 3: pickupThree -> shootThreePos
            driveShootEnd;      // Path 4: shootThreePos -> endPos

    public void buildPaths() {
        // Path 1: start -> shoot preload
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shootPos))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                .build();

        // Path 2: shoot pos -> pickup 3 balls
        driveShootPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, pickupThree))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 3: pickup 3 -> shoot 3 pos (heading sweeps 28 -> 138)
        drivePickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupThree, shootThreePos))
                .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(138))
                .build();

        // Path 4: shoot 3 pos -> end pos
        driveShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootThreePos, endPos))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                // Wait for path to finish + 5 sec spindexer time
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO: stop flywheel/spindexer here
                    follower.followPath(driveShootPickup, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_PICKUPTHREE);
                }
                break;

            case DRIVE_SHOOTPOS_PICKUPTHREE:
                if (!follower.isBusy()) {
                    // TODO: intake logic here
                    follower.followPath(drivePickupShoot, true);
                    setPathState(PathState.DRIVE_PICKUPTHREE_SHOOTTHREE);
                }
                break;

            case DRIVE_PICKUPTHREE_SHOOTTHREE:
                if (!follower.isBusy()) {
                    // TODO: start flywheel/spindexer here
                    setPathState(PathState.SHOOT_THREE);
                }
                break;

            case SHOOT_THREE:
                // 4 seconds for spindexer to shoot the 3 balls
                if (pathTimer.getElapsedTimeSeconds() > 4) {
                    // TODO: stop flywheel/spindexer here
                    follower.followPath(driveShootEnd, true);
                    setPathState(PathState.DRIVE_SHOOTTHREE_ENDPOS);
                }
                break;

            case DRIVE_SHOOTTHREE_ENDPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("All paths complete!");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO: init other mechanisms here
        buildPaths();
        follower.setPose(startPos);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}