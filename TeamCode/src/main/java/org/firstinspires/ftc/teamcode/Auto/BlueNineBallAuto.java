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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@Autonomous(name = "9BallAuto", group = "Autonomous")
@Configurable
public class BlueNineBallAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer pathTimer;

    public static double SHOOT_WAIT_SEC   = 4.0;  // stand still before flicking, lets flywheel spin up
    public static double PICKUP_WAIT_SEC  = 1.0;  // settle time after arriving at pickup
    public static double FLICK_UP_SEC     = 0.9;
    public static double FLICK_DOWN_SEC   = 1.5;
    public static double FLYWHEEL_RPM     = 1500; // TODO: real tuned value

    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final Timer flickTimer = new Timer();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.486, 122.312, Math.toRadians(138)));
        paths = new Paths(follower);
        pathTimer = new Timer();
        pathState = 0;

        Intake.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDownAggressive();
        Turret.INSTANCE.initialize(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.get(Servo.class, "spinServo"),
                hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor")
        );

        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
        Turret.INSTANCE.setHoodPosition(1.0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE); // force known state before scanning
        for (Spindexer.Position pos : Spindexer.Position.values()) {
            Spindexer.INSTANCE.setToPosition(pos);
            try {
                Thread.sleep(400);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            Spindexer.INSTANCE.setColor(pos, Spindexer.INSTANCE.readCurrentColor());
        }
        Spindexer.INSTANCE.periodic();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Spindexer Mode", Spindexer.INSTANCE.getPositionType());
        panelsTelemetry.debug("Spindexer Empty", Spindexer.INSTANCE.getEmpty());
        panelsTelemetry.update(telemetry);
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ════════════════════ SHOOT PRELOAD ════════════════════
            case 0: // drive to preload shoot pos
                follower.followPath(paths.ShootPreload, true);
                setPathState(1);
                break;

            case 1: // arrived - switch spindexer to SHOOT, spin up flywheel
                if (!follower.isBusy()) {
                    enterShootMode();
                    setPathState(2);
                }
                break;

            case 2: // stand still ~4 sec letting flywheel spin up
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    setPathState(3);
                }
                break;

            case 3: // fire all balls in spindexer
                fireAllBalls();
                if (Spindexer.INSTANCE.getEmpty() && flickState == FlickState.IDLE) {
                    exitShootMode(); // back to intake pos, flywheel off
                    follower.followPath(paths.PickupThree, true);
                    setPathState(10);
                }
                break;

            // ════════════════════ PICKUP THREE ════════════════════
            case 10: // driving + intaking
                runIntakeDwell();
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // settle ~1 sec before moving on
                runIntakeDwell();
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_WAIT_SEC) {
                    Intake.INSTANCE.idle();
                    follower.followPath(paths.Shoot3, true);
                    setPathState(20);
                }
                break;

            // ════════════════════ SHOOT3 ════════════════════
            case 20: // arrived - switch to SHOOT, spin up flywheel
                if (!follower.isBusy()) {
                    enterShootMode();
                    setPathState(21);
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    setPathState(22);
                }
                break;

            case 22:
                fireAllBalls();
                if (Spindexer.INSTANCE.getEmpty() && flickState == FlickState.IDLE) {
                    exitShootMode();
                    follower.followPath(paths.DriveToNine, true);
                    setPathState(30);
                }
                break;

            // ════════════════════ DRIVE TO NINE (idle travel) ════════════════════
            case 30: // just drive, mechanisms already idle from exitShootMode()
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupNine, true);
                    setPathState(31);
                }
                break;

            // ════════════════════ PICKUP NINE ════════════════════
            case 31: // driving + intaking (this leg does the actual pickup)
                runIntakeDwell();
                if (!follower.isBusy()) {
                    setPathState(32);
                }
                break;

            case 32:
                runIntakeDwell();
                if (pathTimer.getElapsedTimeSeconds() > PICKUP_WAIT_SEC) {
                    Intake.INSTANCE.idle();
                    follower.followPath(paths.ShootNineAndEnd, true);
                    setPathState(40);
                }
                break;

            // ════════════════════ SHOOT NINE AND END ════════════════════
            case 40: // arrived - switch to SHOOT, spin up flywheel
                if (!follower.isBusy()) {
                    enterShootMode();
                    setPathState(41);
                }
                break;

            case 41:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_WAIT_SEC) {
                    setPathState(42);
                }
                break;

            case 42:
                fireAllBalls();
                if (Spindexer.INSTANCE.getEmpty() && flickState == FlickState.IDLE) {
                    // no extra movement, just stop here
                    Turret.INSTANCE.setVelocity(0);
                    setPathState(99);
                }
                break;

            case 99: // done, sit still
                panelsTelemetry.debug("Status", "All 9 balls scored - Auto Complete");
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────
    private void enterShootMode() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        int next = nextShootSlot();
        if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
        Intake.INSTANCE.idle();
        Turret.INSTANCE.setVelocity(FLYWHEEL_RPM);
    }

    private void exitShootMode() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        int free = Spindexer.INSTANCE.freePosition();
        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
        Turret.INSTANCE.setVelocity(0); // idle flywheel to save power
    }

    /**
     * Fires one ball per call-cycle via the flick state machine.
     * Call this every loop while in a "firing" state; it handles its own timing.
     */
    private void fireAllBalls() {
        if (Spindexer.INSTANCE.getEmpty()) return;

        if (flickState == FlickState.IDLE) {
            Transfer.INSTANCE.transferUpAggressive();
            flickTimer.resetTimer();
            flickState = FlickState.WAIT_UP;
            return;
        }

        switch (flickState) {
            case WAIT_UP:
                if (flickTimer.getElapsedTimeSeconds() >= FLICK_UP_SEC) {
                    Transfer.INSTANCE.transferDownAggressive();
                    flickTimer.resetTimer();
                    flickState = FlickState.WAIT_DOWN;
                }
                break;
            case WAIT_DOWN:
                if (flickTimer.getElapsedTimeSeconds() >= FLICK_DOWN_SEC) {
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    Spindexer.INSTANCE.periodic();
                    flickState = FlickState.IDLE;

                    if (!Spindexer.INSTANCE.getEmpty()) {
                        int next = nextShootSlot();
                        if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                    }
                }
                break;
            default: break;
        }
    }

    private boolean dwelling = false;
    private final Timer dwellTimer = new Timer();
    public static double INTAKE_DWELL_SEC = 0.3;

    private void runIntakeDwell() {
        Intake.INSTANCE.on();
        if (Spindexer.INSTANCE.getPositionType() != Spindexer.PositionType.INTAKE) return;

        if (Spindexer.INSTANCE.getBallAtPosition()[Spindexer.INSTANCE.getPosition().ordinal()]
                != Spindexer.DetectedColor.EMPTY) {
            dwelling = false;
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            return;
        }
        if (!dwelling) {
            dwelling = true;
            dwellTimer.resetTimer();
        } else if (dwellTimer.getElapsedTimeSeconds() >= INTAKE_DWELL_SEC) {
            Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
            Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
            dwelling = false;
            Spindexer.INSTANCE.periodic();
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
        }
    }

    private int nextShootSlot() {
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();
        for (int i = 0; i < slots.length; i++)
            if (slots[i] != Spindexer.DetectedColor.EMPTY) return i;
        return -1;
    }

    public static class Paths {
        public PathChain ShootPreload, PickupThree, Shoot3, DriveToNine, PickupNine, ShootNineAndEnd;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.486, 122.312), new Pose(60.162, 83.168)))
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138)).build();
            PickupThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.162, 83.168), new Pose(18.442, 83.465)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.442, 83.465), new Pose(52.233, 109.233)))
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138)).build();
            DriveToNine = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(52.233, 109.233), new Pose(44.581, 59.651)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            PickupNine = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(44.581, 59.651), new Pose(18.930, 59.721)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            ShootNineAndEnd = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.930, 59.721), new Pose(49.884, 113.581)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138)).build();
        }
    }
}