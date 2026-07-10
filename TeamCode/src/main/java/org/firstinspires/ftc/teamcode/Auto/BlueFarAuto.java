package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@Autonomous(name = "BlueFarAuto", group = "Autonomous")
@Configurable // Panels
public class BlueFarAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer pathTimer;

    public static double SHOOT_WAIT_SEC = 2.5; // 2-3 sec stop for shooting

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);
        pathTimer = new Timer();
        pathState = 0;

        // TODO: init mechanisms here
        // Intake.INSTANCE.init(hardwareMap);
        // Transfer.INSTANCE.initialize(hardwareMap);
        // Turret.INSTANCE.initialize(hardwareMap);
        // Spindexer.INSTANCE.initialize(spinServo, leftColorSensor, rightColorSensor);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // TODO: subsystem periodics
        // Turret.INSTANCE.periodic();
        // Spindexer.INSTANCE.periodic();

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Path Timer", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ════════════ STAND STILL AT START, SHOOT PRELOAD ════════════
            case 0:
                // TODO: enterShootMode(); spin up flywheel
                setPathState(1);
                break;

            case 1: // stand still 2-3 sec to fire preload
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    // TODO: fireAllBalls() / trigger flick sequence
                    // TODO: exitShootMode();
                    follower.followPath(paths.DriveToPickup3, true);
                    setPathState(2);
                }
                break;

            // ════════════ DRIVE (56,36 -> 19.95,35.169) Tangential ════════════
            case 2:
                // TODO: Intake.INSTANCE.on(); (can start intake during this travel leg)
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup3, true);
                    setPathState(3);
                }
                break;

            // ════════════ PICKUP3 (19.95,35.169 -> 55.692,7.174) Linear -35->108 ════════════
            case 3:
                if (!follower.isBusy()) {
                    // TODO: Intake.INSTANCE.idle();
                    follower.followPath(paths.Shoot3, true);
                    setPathState(4);
                }
                break;

            // ════════════ SHOOT3 (55.692,7.174 -> 7.116,7.532) Tangential ════════════
            case 4:
                if (!follower.isBusy()) {
                    // TODO: enterShootMode();
                    setPathState(5);
                }
                break;

            case 5: // stand still 2-3 sec to fire
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    // TODO: fireAllBalls()
                    // TODO: exitShootMode();
                    follower.followPath(paths.Pickup6, true);
                    setPathState(6);
                }
                break;

            // ════════════ PICKUP6 (7.116,7.532 -> 58.339,7.648) Linear 0->120 ════════════
            case 6:
                if (!follower.isBusy()) {
                    // TODO: Intake.INSTANCE.idle();
                    follower.followPath(paths.Shoot6, true);
                    setPathState(7);
                }
                break;

            // ════════════ SHOOT6 (58.339,7.648 -> 7.671,12.859) Tangential ════════════
            case 7:
                if (!follower.isBusy()) {
                    // TODO: enterShootMode();
                    setPathState(8);
                }
                break;

            case 8: // stand still 2-3 sec to fire
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    // TODO: fireAllBalls()
                    // TODO: exitShootMode();
                    follower.followPath(paths.Pickup9, true);
                    setPathState(9);
                }
                break;

            // ════════════ PICKUP9 (7.671,12.859 -> 58.767,9.754) Linear 0->120 ════════════
            case 9:
                if (!follower.isBusy()) {
                    // TODO: Intake.INSTANCE.idle();
                    follower.followPath(paths.Shoot9, true);
                    setPathState(10);
                }
                break;

            // ════════════ SHOOT9 (58.767,9.754 -> 59.183,29.396) Linear 90->90 ════════════
            case 10:
                if (!follower.isBusy()) {
                    // TODO: enterShootMode();
                    setPathState(11);
                }
                break;

            case 11: // stand still 2-3 sec to fire
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    // TODO: fireAllBalls()
                    // TODO: exitShootMode();
                    follower.followPath(paths.Leave, true);
                    setPathState(12);
                }
                break;

            // ════════════ LEAVE - no extra movement needed, this is just it driving off ════════════
            case 12:
                if (!follower.isBusy()) {
                    setPathState(99);
                }
                break;

            case 99: // Done
                panelsTelemetry.debug("Status", "BlueFarAuto Complete");
                break;
        }
    }

    public static class Paths {
        public PathChain DriveToPickup3, Pickup3, Shoot3, Pickup6, Shoot6, Pickup9, Shoot9, Leave;

        public Paths(Follower follower) {
            DriveToPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 36.000), new Pose(19.950, 35.169)))
                    .setTangentHeadingInterpolation()
                    .build();

            Pickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.950, 35.169), new Pose(55.692, 7.174)))
                    .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(108))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.692, 7.174), new Pose(7.116, 7.532)))
                    .setTangentHeadingInterpolation()
                    .build();

            Pickup6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(7.116, 7.532), new Pose(58.339, 7.648)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                    .build();

            Shoot6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.339, 7.648), new Pose(7.671, 12.859)))
                    .setTangentHeadingInterpolation()
                    .build();

            Pickup9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(7.671, 12.859), new Pose(58.767, 9.754)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                    .build();

            Shoot9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.767, 9.754), new Pose(59.183, 29.396)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.183, 29.396), new Pose(59.183, 29.396)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}