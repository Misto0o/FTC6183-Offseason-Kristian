package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "BlueNineBall", group = "Autonomous")
@Configurable // Panels
public class BlueNineBallAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer pathTimer;

    private enum FlickState {WAIT_RPM, FLICK_UP, FLICK_DOWN}

    private FlickState flickState = FlickState.WAIT_RPM;
    private final Timer flickTimer = new Timer();
    private int ballsShot = 0;

    private boolean dwelling = false;
    private final Timer dwellTimer = new Timer();

    // ── Spindexer settle tracking (mirrors Teleop) ──────────────────────────
    // Prevents the fork from firing on a slot the spindexer hasn't actually
    // finished physically arriving at yet.
    public static double SPINDEXER_SETTLE_SEC = 0.25;
    private boolean spindexerSettled = true;
    private final Timer spindexerSettleTimer = new Timer();
    private int lastCommandedSlot = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.485549132947973, 122.31213872832369, Math.toRadians(138)));
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
        // Pre-scan preloaded balls so they are properly stamped.
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

        // Ensure starting in INTAKE pos as requested
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

        // Track settle timer every loop, same as Teleop.
        if (!spindexerSettled && spindexerSettleTimer.getElapsedTimeSeconds() >= SPINDEXER_SETTLE_SEC) {
            spindexerSettled = true;
        }

        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Spindexer Full", Spindexer.INSTANCE.getFull());
        panelsTelemetry.debug("Balls Shot", ballsShot);
        panelsTelemetry.update(telemetry);
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /**
     * Commands the spindexer to the given slot and marks it as unsettled,
     * resetting the settle timer -- mirrors Teleop's slot-change tracking.
     */
    private void commandSlot(int slot) {
        if (slot != lastCommandedSlot) {
            lastCommandedSlot = slot;
            spindexerSettled = false;
            spindexerSettleTimer.resetTimer();
        }
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[slot]);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Drive to shoot preload
                follower.followPath(paths.ShootPreload, true);

                // Switch to SHOOT mode and rev flywheel
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                int preloadSlot = Spindexer.INSTANCE.filledPosition();
                if (preloadSlot != -1) commandSlot(preloadSlot);
                Turret.INSTANCE.setVelocity(1075);

                setPathState(1);
                break;

            case 1: // Wait for path to finish, then shoot preload
                if (!follower.isBusy()) {
                    fireBallsLogic(2);
                }
                break;

            case 2: // Setup PickupThree
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) commandSlot(free);

                Intake.INSTANCE.on();
                follower.followPath(paths.PickupThree, true);
                setPathState(3);
                break;

            case 3: // Execute PickupThree
                runIntakeDwell();
                // Wait until path is done AND 3 balls are picked up
                if (!follower.isBusy() && Spindexer.INSTANCE.getFull()) {
                    Intake.INSTANCE.idle();

                    // Switch to SHOOT mode and rev flywheel
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    int filled3 = Spindexer.INSTANCE.filledPosition();
                    if (filled3 != -1) commandSlot(filled3);
                    Turret.INSTANCE.setVelocity(1075);

                    follower.followPath(paths.Shoot3, true);
                    setPathState(4);
                }
                break;

            case 4: // Wait for path to finish, then shoot 3
                if (!follower.isBusy()) {
                    fireBallsLogic(5);
                }
                break;

            case 5: // Setup DriveToNine
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                free = Spindexer.INSTANCE.freePosition();
                if (free != -1) commandSlot(free);

                Intake.INSTANCE.on();
                follower.followPath(paths.DriveToNine, true);
                setPathState(6);
                break;

            case 6: // Drive toward last set of balls
                runIntakeDwell();
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickupNine, true);
                    setPathState(7);
                }
                break;

            case 7: // Execute PickupNine
                runIntakeDwell();
                if (!follower.isBusy() && Spindexer.INSTANCE.getFull()) {
                    Intake.INSTANCE.idle();

                    // Switch to SHOOT mode and rev flywheel
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    int filled9 = Spindexer.INSTANCE.filledPosition();
                    if (filled9 != -1) commandSlot(filled9);
                    Turret.INSTANCE.setVelocity(1075);

                    follower.followPath(paths.ShootNineAndEnd, true);
                    setPathState(8);
                }
                break;

            case 8: // Wait for path to finish, then shoot last 3
                if (!follower.isBusy()) {
                    fireBallsLogic(9);
                }
                break;

            case 9: // Done
                Turret.INSTANCE.setVelocity(0);
                panelsTelemetry.debug("Status", "All 9 balls scored - Auto Complete");
                break;
        }
    }

    /**
     * Logic to shoot exactly 3 balls. Handles revving, flicking up/down aggressively,
     * waiting for the transfer to clear, and indexing the spindexer.
     * CHANGED: no longer assumes sequential POSITION_ONE -> TWO -> THREE. Always
     * asks the spindexer which slot is actually filled, and won't fire until the
     * spindexer has settled at that slot (mirrors Teleop's spindexerSettled gate).
     */
    private void fireBallsLogic(int nextPathState) {
        if (ballsShot >= 3 || Spindexer.INSTANCE.getEmpty()) {
            Turret.INSTANCE.setVelocity(0);
            ballsShot = 0;
            flickState = FlickState.WAIT_RPM;
            setPathState(nextPathState);
            return;
        }

        switch (flickState) {
            case WAIT_RPM:
                if (Turret.INSTANCE.getVelocity() >= 1050) {
                    // Ask the spindexer what's actually loaded, don't assume a slot.
                    int filled = Spindexer.INSTANCE.filledPosition();
                    if (filled != -1) commandSlot(filled);

                    // Don't fire until the spindexer has actually finished arriving.
                    if (!spindexerSettled) break;

                    Transfer.INSTANCE.transferUpAggressive();
                    flickTimer.resetTimer();
                    flickState = FlickState.FLICK_UP;
                }
                break;

            case FLICK_UP:
                // Re-assert every loop to hold max power against friction
                Transfer.INSTANCE.transferUpAggressive();
                if (flickTimer.getElapsedTimeSeconds() > 1.0) { // was 0.5
                    Transfer.INSTANCE.transferDownAggressive();
                    flickTimer.resetTimer();
                    flickState = FlickState.FLICK_DOWN;
                }
                break;

            case FLICK_DOWN:
                // Wait about 1.4 seconds so it's fully down
                if (flickTimer.getElapsedTimeSeconds() > 1.4) {
                    // Mark slot as empty now that ball is fired
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    Spindexer.INSTANCE.periodic();

                    ballsShot++;
                    // No more manual Position.next() -- next WAIT_RPM pass will
                    // call filledPosition() again and find whichever slot is
                    // actually still loaded.
                    flickState = FlickState.WAIT_RPM;
                }
                break;
        }
    }

    /**
     * Automatically stamps balls when they enter the intake and moves the spindexer
     * to the next open slot until it is full.
     */
    private void runIntakeDwell() {
        Intake.INSTANCE.on();
        if (Spindexer.INSTANCE.getPositionType() != Spindexer.PositionType.INTAKE) return;

        if (!dwelling) {
            dwelling = true;
            dwellTimer.resetTimer();
            return;
        }

        if (dwellTimer.getElapsedTimeSeconds() >= 0.3) {
            Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
            Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
            dwelling = false;
            Spindexer.INSTANCE.periodic();

            // Only advance if a ball was actually detected
            if (seen != Spindexer.DetectedColor.EMPTY) {
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) commandSlot(free);
            }
            // If EMPTY, stay on current slot and dwell again next loop
        }
    }

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

            // Pickup paths: setZeroPowerAccelerationMultiplier brakes harder so bot
            // slows down and actually rolls over balls instead of flying past them.
            // Higher number = harder braking. Start at 4, tune up if still too fast.
            PickupThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.162, 83.168), new Pose(18.442, 83.465)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            PickupThree.getPath(0).setConstraints(new PathConstraints(0.5, 100, 2, 1));

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
            PickupNine.getPath(0).setConstraints(new PathConstraints(0.5, 100, 2, 1));

            ShootNineAndEnd = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.930, 59.721), new Pose(49.884, 113.581)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138))
                    .build();
        }
    }
}