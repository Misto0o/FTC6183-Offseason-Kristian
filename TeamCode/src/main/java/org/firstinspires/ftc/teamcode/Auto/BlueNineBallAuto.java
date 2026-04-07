package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

@Autonomous(name = "Blue 9 Ball Auto", group = "Auto")
@Configurable
public class BlueNineBallAuto extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    // ── Paths ──────────────────────────────────────────────────────────────
    private PathChain mainChain;
    private PathChain chain2;
    private PathChain chain3;

    // ── State Machine ──────────────────────────────────────────────────────
    private enum State {
        START,
        MAIN_CHAIN,     // moveback → driveToMiddle → pickup3
        CHAIN2,         // Shoot6 → driveToTop → Shoot9
        CHAIN3,         // park/gate
        DONE
    }
    private State state = State.START;

    // ── Wait Timer ─────────────────────────────────────────────────────────
    private double waitTimer = 0;
    private boolean waiting = false;

    // ── Poses ──────────────────────────────────────────────────────────────
    private final Pose startPose       = new Pose(20.968, 124.944, Math.toRadians(135));
    private final Pose moveBack        = new Pose(47.586, 97.649);
    private final Pose driveToMiddle   = new Pose(47.567, 58.558);
    private final Pose pickup3         = new Pose(14.081, 58.386);
    private final Pose shoot6          = new Pose(69.033, 83.801);
    private final Pose driveToTop      = new Pose(13.685, 82.738);
    private final Pose shoot9          = new Pose(60.695, 97.894);
    private final Pose gate            = new Pose(19.257, 69.313);

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
        follower.followPath(mainChain);
        state = State.MAIN_CHAIN;
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

            case MAIN_CHAIN:
                // waiting for moveback → driveToMiddle → pickup3 to finish
                if (!follower.isBusy()) {
                    // TODO: trigger intake/shooter here later
                    follower.followPath(chain2);
                    state = State.CHAIN2;
                }
                break;

            case CHAIN2:
                // waiting for Shoot6 → driveToTop → Shoot9 to finish
                if (!follower.isBusy()) {
                    // TODO: trigger shooter here later
                    if (waitFor(0.8)) {
                        follower.followPath(chain3);
                        state = State.CHAIN3;
                    }
                }
                break;

            case CHAIN3:
                // waiting to reach gate
                if (!follower.isBusy()) {
                    // TODO: trigger intake + final shoot here later
                    state = State.DONE;
                }
                break;

            case DONE:
                // auto finished, nothing to do
                break;
        }
    }

    // ── Path Builder ───────────────────────────────────────────────────────
    private void buildPaths() {
        mainChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, moveBack))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .addPath(new BezierLine(moveBack, driveToMiddle))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(driveToMiddle, pickup3))
                .setTangentHeadingInterpolation()
                .build();

        chain2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, shoot6))
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(140))
                .addPath(new BezierLine(shoot6, driveToTop))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(driveToTop, shoot9))
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(140))
                .build();

        chain3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot9, gate))
                .setTangentHeadingInterpolation()
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