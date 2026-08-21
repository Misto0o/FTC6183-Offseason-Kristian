package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.MatchPattern;
import org.firstinspires.ftc.teamcode.Vision.DistanceSensor;

/**
 * Main Competition TeleOp
 * IMPORTANT: This class coordinates all robot subsystems using a series of state machines.
 * It handles the transition between intake and shoot modes automatically.
 */
@Config
@TeleOp(name = "Teleop", group = "MAIN")
public class Teleop extends OpMode {

    // ── Tunable constants ─────────────────────────────────────────────────────
    // IMPORTANT: Use these to adjust timing and behavior without redeploying code.
    public static double INTAKE_DWELL_SEC     = 0.3;
    public static double FLICK_UP_SEC         = 0.9;   // time transfer stays up
    public static double FLICK_DOWN_SEC       = 1.5;   // settle time after returning down
    public static double SHOT_TIMEOUT_SEC     = 1.5;   // max wait for distance sensor to clear
    public static double REFIRE_DELAY_SEC     = 0.5;   // pause before refiring
    public static double RESCAN_MOVE_SEC      = 0.15;  // time to wait after moving spindexer before reading
    public static double SPINDEXER_SETTLE_SEC = 0.25;  // min time after commanding a SHOOT-mode spindexer move before the fork is allowed to fire — prevents fork landing on a ball mid-spin
    public static double HOOD_OVERRIDE        = -1;  // -1 = use table, 0.0-1.0 = manual
    public static double VELO_OVERRIDE        = -1;  // -1 = use table, >0 = manual cap

    private int shotsFired = 0;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private Aliance alliance = Aliance.BLUE;

    // ── Robot state ───────────────────────────────────────────────────────────
    private enum RobotMode { INTAKE, SHOOT }
    private RobotMode mode = RobotMode.INTAKE;

    // Flywheel: 0=off, 1=spinning up, 2=ready
    private int  flywheelState = 0;
    private boolean rumbled    = false;

    private boolean intakeRunning = false;
    private boolean transferDown  = true;

    private boolean turretLock = false;

    private final ElapsedTime relocTimer = new ElapsedTime();


    // ── Flick state machine ───────────────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Spindexer settle tracking (shoot mode) ────────────────────────────────
    // Tracks whether the spindexer has finished physically arriving at the slot
    // it was last commanded to. triggerFlick() checks this before allowing a fire,
    // so spamming SQUARE can't drop the fork while the spindexer is still spinning.
    private boolean spindexerSettled = true;
    private final ElapsedTime spindexerSettleTimer = new ElapsedTime();
    private int lastCommandedSlot = -1;

    // ── Auto-shoot state machine ──────────────────────────────────────────────
    // CHANGED: Added WAIT_FLYWHEEL state so each ball waits for flywheelState==2
    // before firing. This matches Outreach's behaviour and prevents the "shot too
    // fast" jam where the next ball fired before the wheel had recovered speed.
    private enum ShootState { IDLE, WAIT_FLYWHEEL, CONFIRM, REFIRE_WAIT, NEXT_BALL }
    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ── Intake dwell ──────────────────────────────────────────────────────────
    private boolean dwelling = false;
    private final ElapsedTime dwellTimer = new ElapsedTime();

    // ── Rescan state machine ──────────────────────────────────────────────────
    private enum RescanState { IDLE, MOVING, READING }
    private RescanState rescanState = RescanState.IDLE;
    private int rescanIndex = 0;
    private final ElapsedTime rescanTimer = new ElapsedTime();

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DistanceSensor distanceSensor = new DistanceSensor();

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastTriangle, lastCircle, lastCross, lastSquare;
    private boolean lastLB, lastRB, lastLT, lastRT;
    private boolean lastDU, lastDD, lastDL, lastDR;

    // ─────────────────────────────────────────────────────────────────────────
    // INIT
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // IMPORTANT: Initializing all subsystems and ensuring hardware is in a safe state.
        Drivetrain.getInstance().init(hardwareMap);
        distanceSensor.init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDownAggressive();
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);

        // Reset game-specific logic.
        MatchPattern.reset();

        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        boolean du = gamepad1.dpad_up, dd = gamepad1.dpad_down;
        if (du && !lastDU) alliance = Aliance.BLUE;
        if (dd && !lastDD) alliance = Aliance.RED;
        lastDU = du; lastDD = dd;

        MatchPattern.tryDetect();

        if (gamepad1.cross) {
            Turret.INSTANCE.calibrateToParkedPosition();
            gamepad1.rumbleBlips(2);
        }

        // Auto-set position from MT1 if tag is visible.
        // CHANGED: was getMegaTagPose() (MT2) — switched to getMT1Pose() so the
        // starting pose comes from the same solver as in-match relocalization
        // (getSnapshotPose() in loop() already uses MT1 via Limelight.java).
        // MT1 does not need setRobotOrientation() fed in, but it's harmless to
        // leave the call here in case you ever fall back to MT2.
        Limelight.INSTANCE.setRobotOrientation(Pinpoint.INSTANCE.getHeading());
        double[] mt1 = Limelight.INSTANCE.getMT1Pose();
        if (mt1 != null) {
            Pinpoint.INSTANCE.updatePosition(new Pose2D(
                    DistanceUnit.INCH, mt1[0], mt1[1],
                    AngleUnit.DEGREES, Pinpoint.INSTANCE.getHeading()));
            telemetry.addData("Start Pos (MT1)", String.format("(%.1f, %.1f)", mt1[0], mt1[1]));
        } else {
            // Fall back to alliance corner if no tag visible
            double fallbackX = (alliance == Aliance.BLUE) ? 135.5 : 8.5;
            Pinpoint.INSTANCE.updatePosition(new Pose2D(
                    DistanceUnit.INCH, fallbackX, 9,
                    AngleUnit.DEGREES, Pinpoint.INSTANCE.getHeading()));
            telemetry.addData("Start Pos (fallback)", String.format("(%.1f, 9)", fallbackX));
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pattern", MatchPattern.getPattern());
        telemetry.addLine("Hold ✕ with turret facing FRONT to recalibrate");
        telemetry.addData("Turret Angle", Turret.INSTANCE.getTurretAngle());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // START — scan slots, set pose, pick initial mode
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void start() {
        MatchPattern.reset();
        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
        Limelight.INSTANCE.setRobotOrientation(Pinpoint.INSTANCE.getHeading());
        double[] mt1Start = Limelight.INSTANCE.getAveragedSnapshotPose(5);
        if (mt1Start != null) {
            Pinpoint.INSTANCE.updatePosition(new Pose2D(
                    DistanceUnit.INCH, mt1Start[0], mt1Start[1],
                    AngleUnit.DEGREES, Pinpoint.INSTANCE.getHeading()));
        }
        // IMPORTANT: Perform a pre-match scan of the spindexer to identify pre-loaded balls.
        for (Spindexer.Position pos : Spindexer.Position.values()) {
            Spindexer.INSTANCE.setToPosition(pos);
            try { Thread.sleep(400); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            Spindexer.INSTANCE.setColor(pos, Spindexer.INSTANCE.readCurrentColor());
        }
        Spindexer.INSTANCE.periodic();

        // Switch to appropriate mode based on initial load.
        if (Spindexer.INSTANCE.getFull()) enterShootMode();
        else                              enterIntakeMode();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // LOOP
    // ─────────────────────────────────────────────────────────────────────────
    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // ── Always-on systems ─────────────────────────────────────────────────
        Pinpoint.INSTANCE.periodic();
        if (!MatchPattern.isLocked()) MatchPattern.tryDetect();
        Limelight.INSTANCE.setRobotOrientation(Pinpoint.INSTANCE.getHeading());

        // ── Read inputs ───────────────────────────────────────────────────────
        boolean triangle = gamepad1.triangle;
        boolean circle   = gamepad1.circle;
        boolean cross    = gamepad1.cross;
        boolean square   = gamepad1.square;
        boolean lb       = gamepad1.left_bumper;
        boolean rb       = gamepad1.right_bumper;
        boolean lt       = gamepad1.left_trigger  > 0.3;
        boolean rt       = gamepad1.right_trigger > 0.3;
        boolean du       = gamepad1.dpad_up;
        boolean dd       = gamepad1.dpad_down;
        boolean dl       = gamepad1.dpad_left;
        boolean dr       = gamepad1.dpad_right;

        // ── CIRCLE: full stop / reset ─────────────────────────────────────────
        if (circle && !lastCircle) {
            enterIntakeMode();
            intakeRunning = false;
            Intake.INSTANCE.idle();
            flickState = FlickState.IDLE;
            shootState = ShootState.IDLE;
            transferDown = true;
            Transfer.INSTANCE.transferDownAggressive();
            turretLock = false;
        }

        // ── TRIANGLE: intake toggle / misread recovery ────────────────────────
        // CHANGED: ported straight over from Outreach, replacing the old
        // intakeRunning-branching version. Outreach doesn't bother hunting for a
        // specific free slot on the button press itself — it just drops the
        // spindexer into INTAKE mode and turns the intake on, and lets the dwell
        // loop (also ported below) do the actual re-scan/re-stamp work. That's
        // what makes a second Triangle press fix a misread: the dwell loop
        // unconditionally re-reads and re-stamps whatever slot it's sitting on,
        // instead of trusting the old stored color like Teleop used to.
        if (triangle && !lastTriangle) {
            // Still cancel any active shoot/flick cycle first — Outreach has no
            // equivalent state to cancel, but Teleop does, so keep this safe.
            shootState = ShootState.IDLE;
            flickState = FlickState.IDLE;
            transferDown = true;
            Transfer.INSTANCE.transferDownAggressive();

            mode = RobotMode.INTAKE;
            intakeRunning = true;
            flywheelState = 0;
            rumbled = false;
            Turret.INSTANCE.setVelocity(0);
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        // ── SQUARE: flywheel on/off toggle ────────────────────────────────────
        if (square && !lastSquare && mode == RobotMode.SHOOT) {
            switch (flywheelState) {
                case 0: // OFF → spin up
                    flywheelState = 1;
                    rumbled = false;
                    break;
                case 1: // SPINNING UP → cancel
                    flywheelState = 0;
                    rumbled = false;
                    Turret.INSTANCE.setVelocity(0);
                    break;
                case 2: // READY → fire one ball
                    triggerFlick();
                    break;
            }
        }

        // ── Cross: auto shoot-all-three cycle ─────────────────────────────────
        // CHANGED: Now starts in WAIT_FLYWHEEL instead of immediately firing.
        // The auto-shoot state machine will wait until flywheelState==2 before
        // the first shot, and re-wait after each ball. This eliminates the
        // "too fast" misfires from the old version.
        if (cross && !lastCross
                && mode == RobotMode.SHOOT
                && flywheelState > 0          // flywheel must at least be spinning
                && shootState == ShootState.IDLE
                && flickState == FlickState.IDLE) {

            shootTimer.reset();
            shootState = ShootState.WAIT_FLYWHEEL; // gate on flywheel before first shot
        }

        // ── DPad UP: reset odometry to corner ────────────────────────────────
        // IMPORTANT: Use this to clear odometry drift during the match.
        if (du && !lastDU) {
            Pinpoint.INSTANCE.updatePosition(alliance == Aliance.BLUE
                    ? new Pose2D(DistanceUnit.INCH, 135.5, 9, AngleUnit.DEGREES, 0)
                    : new Pose2D(DistanceUnit.INCH, 8.5,   9, AngleUnit.DEGREES, 180));
            Turret.INSTANCE.zeroAngleOffset();
        }

        if (dd && !lastDD) {
            double[] visionPose = Limelight.INSTANCE.getAveragedSnapshotPose(5);
            double visionYaw = Limelight.INSTANCE.getMT1Yaw();
            if (visionPose != null && visionYaw >= 0) {
                double visionX = visionPose[0];
                double visionY = visionPose[1];
                if (visionX > 1.0 && visionX < 143.0 && visionY > 1.0 && visionY < 143.0) {
                    Pinpoint.INSTANCE.relocalizeFull(visionX, visionY, visionYaw);
                    gamepad1.rumble(1.0, 1.0, 150);
                }
            } else {
                gamepad1.rumble(0.5, 0.0, 400);
            }
            Turret.INSTANCE.zeroAngleOffset();
        }

        // ── DPad LEFT/RIGHT: force mode ───────────────────────────────────────
        if (dl && !lastDL) { enterIntakeMode(); intakeRunning = true; Intake.INSTANCE.on(); }
        if (dr && !lastDR) enterShootMode();

        // ── Left trigger: Re-home spindexer if the mount slipped ──────────────
        if (lt && !lastLT) {
            Spindexer.INSTANCE.togglePositionFlip();

            // Re-command the current mode so every subsystem updates immediately.
            if (mode == RobotMode.SHOOT) {
                enterShootMode();
            } else {
                enterIntakeMode();
            }

            gamepad1.rumbleBlips(2);
        }

        if (rt && !lastRT) {
            turretLock = !turretLock;
            gamepad1.rumbleBlips(turretLock ? 3 : 1);
        }

        // ── Bumpers: manual spindexer step ────────────────────────────────────
        if (lb && !lastLB) { Spindexer.Position.next();     Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()); }
        if (rb && !lastRB) { Spindexer.Position.previous(); Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[rb ? 1 : 0]); }

        // ─────────────────────────────────────────────────────────────────────
        // STATE MACHINES
        // ─────────────────────────────────────────────────────────────────────

        // ── Rescan (kept for backward-compat, no longer triggered by Triangle) ─
        tickRescan();

        // ── Transfer flick ────────────────────────────────────────────────────
        tickFlick();

        // ── Auto-shoot ────────────────────────────────────────────────────────
        tickAutoShoot();

        // ─────────────────────────────────────────────────────────────────────
        // DRIVETRAIN
        // ─────────────────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // ─────────────────────────────────────────────────────────────────────
        // INTAKE LOGIC — runs every tick when in intake mode
        // ─────────────────────────────────────────────────────────────────────
        // CHANGED: ported straight over from Outreach. The old Teleop version
        // pre-checked getBallAtPosition()[...] != EMPTY and, if the stored state
        // already said "occupied," skipped reading entirely and jumped to
        // another free slot — which meant a misread slot stayed wrong forever.
        // Outreach's version always re-reads and re-stamps whatever slot it's
        // currently dwelling on, unconditionally overwriting the old value, so
        // a misread gets corrected the next time the dwell loop revisits it.
        if (mode == RobotMode.INTAKE
                && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && flickState == FlickState.IDLE) {

            if (intakeRunning) {
                // Read color and dwell at current slot before deciding to advance
                Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();

                if (!dwelling) {
                    // Start dwell — stay here so sensor can confirm what it sees
                    dwelling = true;
                    dwellTimer.reset();
                } else if (dwellTimer.seconds() >= INTAKE_DWELL_SEC) {
                    // Dwell complete — stamp the confirmed color then advance
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                    dwelling = false;
                    Spindexer.INSTANCE.periodic(); // recompute full/empty after stamp

                    if (Spindexer.INSTANCE.getFull()) {
                        enterShootMode();
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1)
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                }
                // While dwelling: don't move, just keep reading
            } else {
                dwelling = false;
            }
        }

        // ─────────────────────────────────────────────────────────────────────
        // TURRET — always tracking goal when in shoot mode
        // ─────────────────────────────────────────────────────────────────────
        double px = Pinpoint.INSTANCE.getPosX();
        double py = Pinpoint.INSTANCE.getPosY();
        double targetVelo;
        double targetHood;

        int goalId = (alliance == Aliance.BLUE) ? Limelight.BLUE_GOAL_ID : Limelight.RED_GOAL_ID;

        if (mode == RobotMode.SHOOT) {
            // IMPORTANT: Automated aiming logic based on robot position.
            if (!turretLock) {
                Turret.INSTANCE.aimAtGoal(alliance, goalId);
            } else {
                Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
            }

            // IMPORTANT: Lookup table values for optimal shot.
            targetVelo = Turret.INSTANCE.distanceToVelocity(px, py, alliance);
            targetHood = Turret.INSTANCE.distanceToPosition(px, py, alliance);

            // Manual overrides for testing.
            if (VELO_OVERRIDE > 0) targetVelo = VELO_OVERRIDE;
            if (HOOD_OVERRIDE >= 0) targetHood = HOOD_OVERRIDE;

            Turret.INSTANCE.setHoodPosition(targetHood);

            if (flywheelState > 0) Turret.INSTANCE.setVelocity(targetVelo);
            else                   Turret.INSTANCE.setVelocity(0);

            // Advance flywheel state once up to speed.
            if (flywheelState == 1 && Turret.INSTANCE.isSettled() && !rumbled) {
                gamepad1.rumbleBlips(1);
                rumbled       = true;
                flywheelState = 2;
            }

            // Keep spindexer pointed at next ball to shoot.
            // CHANGED: now also tracks settle time — every time a NEW slot is
            // commanded, spindexerSettled goes false and the timer resets. This
            // is what triggerFlick() checks before allowing a fire, so SQUARE
            // spammed right after a shot can't drop the fork mid-spin.
            if (flickState == FlickState.IDLE) {
                if (Spindexer.INSTANCE.getEmpty()) {
                    enterIntakeMode();
                } else {
                    int next = nextShootSlot();
                    if (next != -1) {
                        if (next != lastCommandedSlot) {
                            lastCommandedSlot = next;
                            spindexerSettled = false;
                            spindexerSettleTimer.reset();
                        }
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                    }
                }
            }
            if (!spindexerSettled && spindexerSettleTimer.seconds() >= SPINDEXER_SETTLE_SEC) {
                spindexerSettled = true;
                gamepad1.rumbleBlips(1); // small blip: spindexer settled, fork is clear to fire
            }
        } else {
            // Intake mode — turret parked.
            Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
            Turret.INSTANCE.setVelocity(0);
            Turret.INSTANCE.setHoodPosition(1.0);
        }

        Turret.INSTANCE.periodic();

        // ── Rumble feedback: ball detected ────────────────────────────────────
        if (intakeRunning) {
            Spindexer.DetectedColor detected = Spindexer.INSTANCE.readCurrentColor();
            if      (detected == Spindexer.DetectedColor.GREEN)  gamepad1.rumble(1.0, 0.0, 200);
            else if (detected == Spindexer.DetectedColor.PURPLE) gamepad1.rumble(0.0, 1.0, 200);
        }

        boolean isStill = Math.abs(gamepad1.left_stick_y) < 0.05
                && Math.abs(gamepad1.left_stick_x) < 0.05
                && Math.abs(gamepad1.right_stick_x) < 0.05;

        if (mode == RobotMode.INTAKE && relocTimer.seconds() > 0.5 && isStill) {
            double[] llPose = Limelight.INSTANCE.getSnapshotPose();

            if (llPose != null) {
                double llX = llPose[0];
                double llY = llPose[1];

                if (llX > 1 && llX < 143 && llY > 1 && llY < 143) {
                    double ppX = Pinpoint.INSTANCE.getPosX();
                    double ppY = Pinpoint.INSTANCE.getPosY();

                    double error = Math.hypot(llX - ppX, llY - ppY);

                    if (error < 20) {
                        double blendedX = 0.95 * ppX + 0.05 * llX;
                        double blendedY = 0.95 * ppY + 0.05 * llY;

                        Pinpoint.INSTANCE.relocalizePositionFromTag(blendedX, blendedY);
                        relocTimer.reset();
                    }
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────
// MATCH TELEMETRY
// ─────────────────────────────────────────────────────────────────────

        double llDist = Limelight.INSTANCE.distanceFromTag(goalId);

        Spindexer.DetectedColor[] balls = Spindexer.INSTANCE.getBallAtPosition();

        int ballCount = 0;
        for (Spindexer.DetectedColor c : balls) {
            if (c != Spindexer.DetectedColor.EMPTY) {
                ballCount++;
            }
        }

        String flywheelStatus =
                flywheelState == 0 ? "OFF" :
                        flywheelState == 1 ? "SPINNING" :
                                "READY";

        boolean readyToShoot =
                flywheelState == 2 &&
                        Turret.INSTANCE.isSettled();

        double[] mt1 = Limelight.INSTANCE.getMT1Pose();

        telemetry.addLine("══════ MATCH STATUS ══════");
        telemetry.addData("Mode", mode);
        telemetry.addData("Pattern",
                MatchPattern.getPattern() +
                        (MatchPattern.isLocked() ? " 🔒" : ""));
        telemetry.addData("Ready", readyToShoot ? "✓ YES" : "NO");

        telemetry.addLine("──── LOCALIZATION ─────");
        telemetry.addData("Pinpoint",
                String.format("(%.1f, %.1f) %.1f°",
                        px, py,
                        Math.toDegrees(Pinpoint.INSTANCE.getHeading())));

        if (mt1 != null) {
            telemetry.addData("MT1",
                    String.format("(%.1f, %.1f)",
                            mt1[0], mt1[1]));
        } else {
            telemetry.addData("MT1", "NO DATA");
        }

        telemetry.addData("Drift",
                mt1 != null
                        ? String.format("%.1f, %.1f",
                        px - mt1[0],
                        py - mt1[1])
                        : "N/A");

        telemetry.addLine("──── SHOOTER ─────");
        telemetry.addData("Flywheel", flywheelStatus);
        telemetry.addData("RPM",
                String.format("%d / %d",
                        (int) Turret.INSTANCE.getVelocity(),
                        (int) Turret.INSTANCE.distanceToVelocity(px, py, alliance)));
        telemetry.addData("AutoShoot", shootState);
        telemetry.addData("Distance",
                llDist > 0
                        ? String.format("%.1f in", llDist)
                        : "NO TARGET");

        telemetry.addLine("──── TURRET ─────");
        telemetry.addData("Target",
                String.format("%.1f°", Turret.INSTANCE.getTurretAngleSet()));
        telemetry.addData("Actual",
                String.format("%.1f°", Turret.INSTANCE.getTurretAngle()));
        telemetry.addData("Offset",
                String.format("%.1f°", Turret.INSTANCE.getTurretOffSet()));
        telemetry.addData("Settled",
                Turret.INSTANCE.isSettled() ? "✓" : "NO");

        telemetry.addData("Balls",   String.format("%d/3  [%s|%s|%s]",
                ballCount,
                balls[0] == Spindexer.DetectedColor.EMPTY ? "__" : balls[0].name().substring(0,2),
                balls[1] == Spindexer.DetectedColor.EMPTY ? "__" : balls[1].name().substring(0,2),
                balls[2] == Spindexer.DetectedColor.EMPTY ? "__" : balls[2].name().substring(0,2)));

        telemetry.addData("Shots",   String.format("%d fired | Auto: %s", shotsFired, shootState));

        telemetry.addData("Flipped",
                Spindexer.positionsFlipped ? "YES" : "NO");

        telemetry.update();

        // ── Edge detect bookkeeping ───────────────────────────────────────────
        lastTriangle = triangle; lastCircle = circle;
        lastCross    = cross;    lastSquare = square;
        lastLB = lb; lastRB = rb; lastLT = lt; lastRT = rt;
        lastDU = du; lastDD = dd; lastDL = dl; lastDR = dr;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // STATE MACHINE TICKS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * IMPORTANT: Cycles through all spindexer slots and updates the internal color map.
     * Automatically transitions to shooting mode if full, or resumes intaking if slots remain.
     * NOTE: This is no longer triggered by Triangle — it's kept for any future use.
     */
    private void tickRescan() {
        switch (rescanState) {
            case MOVING:
                if (rescanTimer.seconds() >= RESCAN_MOVE_SEC) {
                    Spindexer.INSTANCE.setColor(
                            Spindexer.Position.values()[rescanIndex],
                            Spindexer.INSTANCE.readCurrentColor());
                    rescanState = RescanState.READING;
                    rescanTimer.reset();
                }
                break;
            case READING:
                rescanIndex++;
                if (rescanIndex >= Spindexer.Position.values().length) {
                    Spindexer.INSTANCE.periodic();

                    if (Spindexer.INSTANCE.getFull()) {
                        enterShootMode();
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) {
                            mode = RobotMode.INTAKE;
                            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                            intakeRunning = true;
                            Intake.INSTANCE.on();
                            dwelling = false;
                        } else {
                            enterIntakeMode();
                        }
                    }
                    rescanState = RescanState.IDLE;
                } else {
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[rescanIndex]);
                    rescanTimer.reset();
                    rescanState = RescanState.MOVING;
                }
                break;
            default: break;
        }
    }

    /**
     * IMPORTANT: Coordinates the transfer forks and flywheel recovery after a shot.
     */
    private void tickFlick() {
        switch (flickState) {
            case WAIT_UP:
                if (flickTimer.seconds() >= FLICK_UP_SEC) {
                    Transfer.INSTANCE.transferDownAggressive();
                    flickTimer.reset();
                    flickState = FlickState.WAIT_DOWN;
                }
                break;
            case WAIT_DOWN:
                if (flickTimer.seconds() >= FLICK_DOWN_SEC) {
                    // Mark current slot empty.
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    shotsFired++; // track how many balls have actually been fired

                    // IMPORTANT: Position type toggling ensures the spindexer centers correctly for the next ball.
                    int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    Spindexer.INSTANCE.periodic();

                    flickTimer.reset();
                    flickState   = FlickState.IDLE;
                    transferDown = true;

                    // CHANGED: Reset flywheel to spinning-up so tickAutoShoot's
                    // WAIT_FLYWHEEL gate forces a proper speed confirmation before
                    // the next ball fires. This is the key fix for the "too fast" issue.
                    flywheelState = 1;
                    rumbled       = false;
                }
                break;
            default: break;
        }
    }

    /**
     * IMPORTANT: Automates the sequence of shooting multiple balls.
     * CHANGED: Added WAIT_FLYWHEEL state — each ball now waits for the flywheel
     * to confirm ready (flywheelState==2) before firing. Matches Outreach behaviour.
     */
    private void tickAutoShoot() {
        switch (shootState) {
            // ── NEW STATE: wait for flywheel to spin up before firing ──────────
            case WAIT_FLYWHEEL:
                // Abort if somehow no balls remain
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootState = ShootState.IDLE;
                    break;
                }
                // Once flywheel is confirmed ready, fire
                if (flywheelState == 2 && flickState == FlickState.IDLE && transferDown) {
                    shootTimer.reset();
                    shootState = ShootState.CONFIRM;
                    triggerFlick();
                }
                break;

            case CONFIRM:
                if (flickState != FlickState.IDLE) break;
                if (distanceSensor.isClear()) {
                    // Ball confirmed gone — queue next ball.
                    shootState = ShootState.NEXT_BALL;
                } else if (shootTimer.seconds() > SHOT_TIMEOUT_SEC) {
                    // Didn't clear in time — assume miss or jam and refire.
                    shootTimer.reset();
                    shootState = ShootState.REFIRE_WAIT;
                }
                break;

            case REFIRE_WAIT:
                if (shootTimer.seconds() >= REFIRE_DELAY_SEC
                        && flickState == FlickState.IDLE
                        && transferDown) {
                    shootTimer.reset();
                    // CHANGED: Go back through WAIT_FLYWHEEL to ensure speed is
                    // confirmed before the refire attempt.
                    shootState = ShootState.WAIT_FLYWHEEL;
                }
                break;

            case NEXT_BALL:
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootState = ShootState.IDLE;
                    break;
                }
                // CHANGED: Gate on WAIT_FLYWHEEL instead of firing directly.
                // This ensures the wheel has re-spun after the previous flick
                // before we fire the next ball.
                if (flickState == FlickState.IDLE && transferDown) {
                    shootState = ShootState.WAIT_FLYWHEEL;
                }
                break;

            default: break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────────────────

    private void enterShootMode() {
        mode          = RobotMode.SHOOT;
        Turret.INSTANCE.zeroAngleOffset();
        intakeRunning = false;
        flywheelState = 1;
        rumbled       = false;
        dwelling      = false;
        shootState    = ShootState.IDLE;
        flickState    = FlickState.IDLE;
        shotsFired = 0; // Reset when in shoot mode
        Intake.INSTANCE.idle();
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        int next = nextShootSlot();
        if (next != -1) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
            // CHANGED: gate the first shot too — entering shoot mode commands a
            // spindexer move just like any other slot change, so it needs the
            // same settle delay before triggerFlick() will allow a fire.
            lastCommandedSlot = next;
            spindexerSettled  = false;
            spindexerSettleTimer.reset();
        }
    }

    private void enterIntakeMode() {
        mode          = RobotMode.INTAKE;
        Turret.INSTANCE.zeroAngleOffset();
        intakeRunning = false;
        flywheelState = 0;
        rumbled       = false;
        dwelling      = false;
        shootState    = ShootState.IDLE;
        flickState    = FlickState.IDLE;
        Turret.INSTANCE.setVelocity(0);
        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
        Turret.INSTANCE.setHoodPosition(1.0);
        Intake.INSTANCE.idle();
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        int free = Spindexer.INSTANCE.freePosition();
        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
    }

    private void triggerFlick() {
        // CHANGED: added spindexerSettled check — fork can't go up until the
        // spindexer has actually finished arriving at the commanded slot, not
        // just whenever flickState happens to be IDLE. This is the fix for the
        // SQUARE-spam jam (fork landing flat on a ball mid-spin).
        if (flickState == FlickState.IDLE && transferDown && spindexerSettled) {
            transferDown  = false;
            Transfer.INSTANCE.transferUpAggressive();
            flickTimer.reset();
            flickState    = FlickState.WAIT_UP;
            // NOTE: flywheelState intentionally NOT reset here — tickFlick's
            // WAIT_DOWN does it after the transfer fully returns, so
            // WAIT_FLYWHEEL always sees a clean state.
        }
    }

    /**
     * IMPORTANT: Determines which ball to fire next based on the field pattern.
     * If the pattern is UNKNOWN, it simply fires the first available ball.
     */
    private int nextShootSlot() {
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();

        if (MatchPattern.isLocked()) {
            Spindexer.DetectedColor[] order;
            switch (MatchPattern.getPattern()) {
                case GPP: order = new Spindexer.DetectedColor[]{ Spindexer.DetectedColor.GREEN,  Spindexer.DetectedColor.PURPLE, Spindexer.DetectedColor.PURPLE }; break;
                case PGP: order = new Spindexer.DetectedColor[]{ Spindexer.DetectedColor.PURPLE, Spindexer.DetectedColor.GREEN,  Spindexer.DetectedColor.PURPLE }; break;
                case PPG: order = new Spindexer.DetectedColor[]{ Spindexer.DetectedColor.PURPLE, Spindexer.DetectedColor.PURPLE, Spindexer.DetectedColor.GREEN  }; break;
                default:  order = null; break;
            }
            if (order != null && shotsFired < order.length) {
                Spindexer.DetectedColor needed = order[shotsFired];
                for (int i = 0; i < slots.length; i++)
                    if (slots[i] == needed) return i;
            }
        }

        // Fallback — fire whatever is available
        for (int i = 0; i < slots.length; i++)
            if (slots[i] != Spindexer.DetectedColor.EMPTY) return i;
        return -1;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
    }
}