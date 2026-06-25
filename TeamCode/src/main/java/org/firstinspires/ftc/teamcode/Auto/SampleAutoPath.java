package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pedro.Constants;


@TeleOp
public class SampleAutoPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private TelemetryManager panelsTelemetry;
    public enum PathState {

        //Startpos --> endpos
        //Drive = movement
        //Shoot = scoring artifacts
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }

    PathState pathState;

    private final Pose startPose = new Pose(123.42857142857146, 122.57851239669421, Math.toRadians(48));
    private final Pose shootPose = new Pose(96.90672963400242, 96.22668240850061, Math.toRadians(270));

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // resets timer and makes new state
                break;
            case SHOOT_PRELOAD:
                // Implement shooting logic here
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter (tune and stuff)
                    telemetry.addLine("Drive complete, ready to shoot! Path one done.");
                }
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
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add other init logic here (mech)

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("path timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
    }
}