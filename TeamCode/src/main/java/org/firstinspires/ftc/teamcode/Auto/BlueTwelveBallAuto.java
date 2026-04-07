package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Blue 12 Ball Auto", group = "Auto")
@Configurable
public class BlueTwelveBallAuto extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    // ── State Machine ──────────────────────────────────────────────────────
    private enum State {
        SCORE_PRELOAD,        // drive to shoot zone
        PICKUP_MIDDLE_ROW,    // sweep middle row
        RETURN_TO_SHOOT_1,    // back to shoot zone (6 ball)
        PICKUP_TOP_ROW,       // sweep top row
        RETURN_TO_SHOOT_2,    // back to shoot zone (9 ball)
        GATE_MOVE_1,          // first gate position
        GATE_MOVE_2,          // second gate position
        RETURN_TO_SHOOT_3,    // back to shoot zone (12 ball)
        DONE
    }
    private State state = State.SCORE_PRELOAD;

    // ── Wait Timer ─────────────────────────────────────────────────────────
    private double waitTimer = 0;
    private boolean waiting = false;

    // ── Poses ──────────────────────────────────────────────────────────────
    private final Pose startPose      = new Pose(24.980, 127.469, Math.toRadians(142));
    private final Pose shootZone      = new Pose(61.531, 84.857);
    private final Pose middleRowEnd   = new Pose(17.837, 59.898);
    private final Pose gatePos1       = new Pose(18.041, 67.816);
    private final Pose gatePos2       = new Pose(13.755, 67.714);
    private final Pose shootZone2     = new Pose(61.531, 72.449);
    private final Pose shootZone3     = new Pose(61.449, 84.918);
    private final Pose topRowEnd      = new Pose(17.837, 84.143);
    private final Pose shootZone4     = new Pose(61.612, 85.122);

    // ── Paths ──────────────────────────────────────────────────────────────
    private PathChain scorePreload;
    private PathChain pickupMiddleRow;
    private PathChain returnToShoot1;
    private PathChain pickupTopRow;
    private PathChain returnToShoot2;
    private PathChain gateMove1;
    private PathChain gateMove2;
    private PathChain returnToShoot3;

    // ──────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        panelsTelemetry.debug("Status", "Waiting for start...");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(scorePreload);
        state = State.SCORE_PRELOAD;
    }

    @Override
    public void loop() {
        follower.update();
        updateStateMachine();

        panelsTelemetry.debug("State", state.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    // ── State Machine ──────────────────────────────────────────────────────
    private void updateStateMachine() {
        switch (state) {

            case SCORE_PRELOAD:
                if (!follower.isBusy()) {
                    // TODO: shoot preload here
                    if (waitFor(0.8)) {
                        follower.followPath(pickupMiddleRow);
                        state = State.PICKUP_MIDDLE_ROW;
                    }
                }
                break;

            case PICKUP_MIDDLE_ROW:
                if (!follower.isBusy()) {
                    // TODO: intake should have collected middle row
                    follower.followPath(returnToShoot1);
                    state = State.RETURN_TO_SHOOT_1;
                }
                break;

            case RETURN_TO_SHOOT_1:
                if (!follower.isBusy()) {
                    // TODO: shoot 3 (6 total)
                    if (waitFor(0.8)) {
                        follower.followPath(pickupTopRow);
                        state = State.PICKUP_TOP_ROW;
                    }
                }
                break;

            case PICKUP_TOP_ROW:
                if (!follower.isBusy()) {
                    // TODO: intake should have collected top row
                    follower.followPath(returnToShoot2);
                    state = State.RETURN_TO_SHOOT_2;
                }
                break;

            case RETURN_TO_SHOOT_2:
                if (!follower.isBusy()) {
                    // TODO: shoot 3 (9 total)
                    if (waitFor(0.8)) {
                        follower.followPath(gateMove1);
                        state = State.GATE_MOVE_1;
                    }
                }
                break;

            case GATE_MOVE_1:
                if (!follower.isBusy()) {
                    follower.followPath(gateMove2);
                    state = State.GATE_MOVE_2;
                }
                break;

            case GATE_MOVE_2:
                if (!follower.isBusy()) {
                    // TODO: intake gate balls
                    follower.followPath(returnToShoot3);
                    state = State.RETURN_TO_SHOOT_3;
                }
                break;

            case RETURN_TO_SHOOT_3:
                if (!follower.isBusy()) {
                    // TODO: shoot 3 (12 total)
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    // ── Path Builder ───────────────────────────────────────────────────────
    private void buildPaths() {

        // drive from start to shoot zone, rotating from 142° to 180°
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootZone))
                .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                .build();

        // curve sweeps wide through middle row (Cheick's original curve - better than a line)
        pickupMiddleRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootZone,
                        new Pose(68.908, 56.918),   // control point, swings wide to sweep balls
                        middleRowEnd
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // straight back to shoot zone facing same direction
        returnToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(middleRowEnd, shootZone3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // sweep top row
        pickupTopRow = follower.pathBuilder()
                .addPath(new BezierLine(shootZone3, topRowEnd))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // back to shoot zone
        returnToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(topRowEnd, shootZone4))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // move into gate position 1
        gateMove1 = follower.pathBuilder()
                .addPath(new BezierLine(shootZone4, gatePos1))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // slide to gate position 2
        gateMove2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePos1, gatePos2))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // return to shoot zone for final 3
        returnToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(gatePos2, shootZone2))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(new BezierLine(shootZone2, shootZone3))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    // ── Wait Helper ────────────────────────────────────────────────────────
    private boolean waitFor(double seconds) {
        if (!waiting) {
            waitTimer = System.currentTimeMillis();
            waiting = true;
        }
        if (System.currentTimeMillis() - waitTimer >= seconds * 1000) {
            waiting = false;
            return true;
        }
        return false;
    }
}