package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.PathConstraints;
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

/**
 * SIMPLIFIED BlueNineBall Auto.
 *
 * Philosophy: this is just Teleop's mech logic (intake dwell/stamp, flick,
 * flywheel wait) wired up to fire automatically instead of on button press,
 * with PathChains driving between spots instead of a joystick. No turret
 * aiming -- turret stays parked at TURRET_PARKED_ANGLE the whole match, same
 * as Teleop when turretLock is on.
 *
 * Flow is just: DRIVE -> INTAKE UNTIL FULL -> DRIVE -> SHOOT 3 -> repeat -> DONE.
 */
@Autonomous(name = "BlueNineBall", group = "Autonomous")
@Configurable // Panels
public class BlueNineBallAuto extends OpMode {

    // ── Tunables (same numbers/spirit as Teleop) ────────────────────────────
    public static double INTAKE_DWELL_SEC     = 0.3;   // matches Teleop.INTAKE_DWELL_SEC
    public static double FLICK_UP_SEC         = 0.9;   // matches Teleop.FLICK_UP_SEC
    public static double FLICK_DOWN_SEC       = 1.5;   // matches Teleop.FLICK_DOWN_SEC
    public static double SPINDEXER_SETTLE_SEC = 0.25;  // matches Teleop.SPINDEXER_SETTLE_SEC
    public static double SHOOT_VELOCITY       = 1075;
    // Max time to spend hunting for the last ball(s) before giving up and
    // shooting whatever's actually in the spindexer. Prevents a missed ball
    // from stalling the whole auto.
    public static double PICKUP_TIMEOUT_SEC   = 3.0;
    // Cap on drivetrain power while crawling through the pickup zone, so the
    // bot doesn't outrun the intake. 1.0 = full speed (used for every other
    // leg). Tune this down further if it's still rolling past balls.
    public static double PICKUP_MAX_POWER     = 0.85;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private Timer pathTimer;

    // ── Simple top-level sequence ────────────────────────────────────────────
    private enum Step {
        DRIVE_TO_PRELOAD_SHOT, SHOOT_PRELOAD,
        DRIVE_TO_PICKUP_3, INTAKE_3, DRIVE_TO_SHOOT_3, SHOOT_3,
        DRIVE_TO_9, PICKUP_9, DRIVE_TO_SHOOT_9, SHOOT_9,
        DONE
    }
    private Step step = Step.DRIVE_TO_PRELOAD_SHOT;

    // ── Intake dwell sub-state (copied straight from Teleop) ────────────────
    private boolean dwelling = false;
    private final Timer dwellTimer = new Timer();

    // ── Flick sub-state (copied straight from Teleop's FlickState) ──────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final Timer flickTimer = new Timer();
    private boolean transferDown = true;

    // ── Shoot-3 bookkeeping ──────────────────────────────────────────────────
    private int ballsShotThisVolley = 0;
    private boolean waitingForFlywheel = false;

    // ── Spindexer settle tracking (copied straight from Teleop) ─────────────
    private boolean spindexerSettled = true;
    private final Timer spindexerSettleTimer = new Timer();
    private int lastCommandedSlot = -1;

    // ── Pickup stall timeout tracking ────────────────────────────────────────
    private final Timer pickupStallTimer = new Timer();
    private boolean pickupStallTimerStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.485549132947973, 122.31213872832369, Math.toRadians(138)));
        paths = new Paths(follower);
        pathTimer = new Timer();

        Intake.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDownAggressive();
        Turret.INSTANCE.initialize(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.get(Servo.class, "spinServo"),
                hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor")
        );

        // Turret parked the whole match -- no aiming in this auto.
        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
        Turret.INSTANCE.setHoodPosition(1.0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Pre-scan preloaded balls, same as Teleop's start().
        for (Spindexer.Position pos : Spindexer.Position.values()) {
            Spindexer.INSTANCE.setToPosition(pos);
            try { Thread.sleep(400); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            Spindexer.INSTANCE.setColor(pos, Spindexer.INSTANCE.readCurrentColor());
        }
        Spindexer.INSTANCE.periodic();

        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);

        // Spin the flywheel up immediately and leave it running the whole
        // match -- no more spin-up/spin-down between volleys. This means by
        // the time we reach each shoot spot the wheel is already at speed,
        // instead of waiting on the gate in runShootVolley().
        Turret.INSTANCE.setVelocity(SHOOT_VELOCITY);

        follower.setMaxPower(1.0);
        follower.followPath(paths.ShootPreload, true);
        step = Step.DRIVE_TO_PRELOAD_SHOT;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

        if (!spindexerSettled && spindexerSettleTimer.getElapsedTimeSeconds() >= SPINDEXER_SETTLE_SEC) {
            spindexerSettled = true;
        }

        runSequence();

        panelsTelemetry.debug("Step", step);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Spindexer Full", Spindexer.INSTANCE.getFull());
        panelsTelemetry.update(telemetry);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Top-level sequence -- just: drive, intake, drive, shoot, repeat.
    // ─────────────────────────────────────────────────────────────────────────
    private void runSequence() {
        switch (step) {

            case DRIVE_TO_PRELOAD_SHOT:
                if (!follower.isBusy()) {
                    startShootVolley();
                    step = Step.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (runShootVolley()) {
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    goToFreeSlotOrStay();
                    Intake.INSTANCE.on();
                    follower.setMaxPower(PICKUP_MAX_POWER);
                    follower.followPath(paths.PickupThree, true);
                    step = Step.DRIVE_TO_PICKUP_3;
                }
                break;

            case DRIVE_TO_PICKUP_3:
                // Run intake logic *while* driving, exactly like Teleop lets you
                // drive and intake at the same time.
                runIntakeDwell();
                if (!follower.isBusy() || Spindexer.INSTANCE.getFull()) {
                    step = Step.INTAKE_3;
                    // Start the stall clock only now -- once we're actually
                    // stationary/hunting, not during the drive over.
                    startPickupStallTimer();
                }
                break;

            case INTAKE_3:
                // Keep dwelling in place until full, in case the path finished
                // before all 3 balls were picked up. If we truly can't find the
                // last ball within PICKUP_TIMEOUT_SEC, stop waiting and shoot
                // whatever we've got rather than stalling the whole auto.
                runIntakeDwell();
                if (Spindexer.INSTANCE.getFull() || pickupStalled()) {
                    Intake.INSTANCE.idle();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.Shoot3, true);
                    step = Step.DRIVE_TO_SHOOT_3;
                }
                break;

            case DRIVE_TO_SHOOT_3:
                if (!follower.isBusy()) {
                    startShootVolley();
                    step = Step.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (runShootVolley()) {
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    goToFreeSlotOrStay();
                    Intake.INSTANCE.on();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.DriveToNine, true);
                    step = Step.DRIVE_TO_9;
                }
                break;

            case DRIVE_TO_9:
                runIntakeDwell();
                if (!follower.isBusy()) {
                    follower.setMaxPower(PICKUP_MAX_POWER);
                    follower.followPath(paths.PickupNine, true);
                    step = Step.PICKUP_9;
                    pickupStallTimerStarted = false; // will lazy-start once path finishes below
                }
                break;

            case PICKUP_9:
                runIntakeDwell();
                // Start the stall clock only once we've stopped actually
                // driving/searching -- not during the travel itself.
                if (!follower.isBusy() && !pickupStallTimerStarted) {
                    startPickupStallTimer();
                }
                if (Spindexer.INSTANCE.getFull() || (!follower.isBusy() && pickupStalled())) {
                    Intake.INSTANCE.idle();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ShootNineAndEnd, true);
                    step = Step.DRIVE_TO_SHOOT_9;
                }
                break;

            case DRIVE_TO_SHOOT_9:
                if (!follower.isBusy()) {
                    startShootVolley();
                    step = Step.SHOOT_9;
                }
                break;

            case SHOOT_9:
                if (runShootVolley()) {
                    Turret.INSTANCE.setVelocity(0);
                    step = Step.DONE;
                }
                break;

            case DONE:
                panelsTelemetry.debug("Status", "All 9 balls scored - Auto Complete");
                break;
        }
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        // Park spindexer in intake mode so teleop inherits a clean state
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooting -- same shape as Teleop: enter shoot mode, spin flywheel, wait
    // for it to reach speed, flick, confirm, repeat until 3 fired or empty.
    // ─────────────────────────────────────────────────────────────────────────
    private void startShootVolley() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        ballsShotThisVolley = 0;
        // Flywheel is already spinning continuously (set once in start()), so
        // this just double-checks it's at speed rather than waiting on a
        // fresh spin-up.
        waitingForFlywheel = true;
        commandSlotToFilled();
    }

    /** @return true once 3 balls are fired (or spindexer runs empty). */
    private boolean runShootVolley() {
        if (ballsShotThisVolley >= 3 || Spindexer.INSTANCE.getEmpty()) {
            // NOTE: flywheel is intentionally left spinning here -- it only
            // gets cut in SHOOT_9 (end of match) or stop().
            flickState = FlickState.IDLE;
            transferDown = true;
            return true;
        }

        // Always re-check which slot is actually loaded (Teleop does the same).
        commandSlotToFilled();

        switch (flickState) {
            case IDLE:
                if (waitingForFlywheel) {
                    if (Turret.INSTANCE.getVelocity() >= SHOOT_VELOCITY - 25 && spindexerSettled) {
                        waitingForFlywheel = false;
                    } else {
                        break;
                    }
                }
                transferDown = false;
                Transfer.INSTANCE.transferUpAggressive();
                flickTimer.resetTimer();
                flickState = FlickState.WAIT_UP;
                break;

            case WAIT_UP:
                Transfer.INSTANCE.transferUpAggressive();
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

                    ballsShotThisVolley++;
                    transferDown = true;
                    flickState = FlickState.IDLE;
                    waitingForFlywheel = true; // re-confirm speed before next ball, like Teleop
                }
                break;
        }
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake dwell -- copied straight from Teleop's per-loop intake block.
    // ─────────────────────────────────────────────────────────────────────────
    private void runIntakeDwell() {
        Intake.INSTANCE.on();
        if (Spindexer.INSTANCE.getPositionType() != Spindexer.PositionType.INTAKE) return;
        if (Spindexer.INSTANCE.getFull()) return;

        Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();

        if (!dwelling) {
            dwelling = true;
            dwellTimer.resetTimer();
            return;
        }

        if (dwellTimer.getElapsedTimeSeconds() >= INTAKE_DWELL_SEC) {
            Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
            dwelling = false;
            Spindexer.INSTANCE.periodic();

            if (Spindexer.INSTANCE.getFull()) return;

            if (seen != Spindexer.DetectedColor.EMPTY) {
                goToFreeSlotOrStay();
            }
            // If EMPTY, stay put and dwell again next loop -- same as Teleop.
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Slot helpers -- wrap Spindexer.setToPosition with the settle-timer
    // tracking, same idea as Teleop's inline lastCommandedSlot logic.
    // ─────────────────────────────────────────────────────────────────────────
    private void goToFreeSlotOrStay() {
        int free = Spindexer.INSTANCE.freePosition();
        if (free != -1) commandSlot(free);
    }

    private void commandSlotToFilled() {
        int filled = Spindexer.INSTANCE.filledPosition();
        if (filled != -1) commandSlot(filled);
    }

    private void startPickupStallTimer() {
        pickupStallTimer.resetTimer();
        pickupStallTimerStarted = true;
    }

    /** @return true once we've spent too long hunting for the last ball(s). */
    private boolean pickupStalled() {
        return pickupStallTimerStarted
                && pickupStallTimer.getElapsedTimeSeconds() >= PICKUP_TIMEOUT_SEC;
    }

    private void commandSlot(int slot) {
        if (slot != lastCommandedSlot) {
            lastCommandedSlot = slot;
            spindexerSettled = false;
            spindexerSettleTimer.resetTimer();
        }
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[slot]);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Paths -- unchanged from the original file.
    // ─────────────────────────────────────────────────────────────────────────
    public static class Paths {
        public PathChain ShootPreload;
        public PathChain PickupThree;
        public PathChain Shoot3;
        public PathChain DriveToNine;
        public PathChain PickupNine;
        public PathChain ShootNineAndEnd;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.486, 122.312), new Pose(60.162, 83.168)))
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                    .build();

            PickupThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.162, 83.168), new Pose(18.442, 83.465)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            // NOTE: actual speed control for this leg is done via
            // follower.setMaxPower(PICKUP_MAX_POWER) in the state machine
            // below, not here -- PathConstraints tunes path-following
            // tolerances, not drivetrain speed.
            PickupThree.getPath(0).setConstraints(new PathConstraints(1.5, 100, 2, 1));
            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.442, 83.465), new Pose(52.233, 109.233)))
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))
                    .build();

            DriveToNine = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(52.233, 109.233), new Pose(44.581, 59.651)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PickupNine = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(44.581, 59.651), new Pose(18.930, 59.721)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            PickupNine.getPath(0).setConstraints(new PathConstraints(1.5, 100, 2, 1));
            ShootNineAndEnd = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.930, 59.721), new Pose(49.884, 113.581)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();
        }
    }
}