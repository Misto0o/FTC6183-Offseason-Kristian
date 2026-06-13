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
import org.firstinspires.ftc.teamcode.Outreach.Outreach;
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
    public static double HOOD_OVERRIDE        = -1;  // -1 = use table, 0.0-1.0 = manual
    public static double VELO_OVERRIDE        = -1;  // -1 = use table, >0 = manual cap

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

    // ── Auto-shoot state machine ──────────────────────────────────────────────
    private enum ShootState { IDLE, CONFIRM, REFIRE_WAIT, NEXT_BALL }
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
        // IMPORTANT: Use this time to select alliance and detect the field pattern.
        boolean du = gamepad1.dpad_up, dd = gamepad1.dpad_down;
        if (du && !lastDU) alliance = Aliance.BLUE;
        if (dd && !lastDD) alliance = Aliance.RED;
        lastDU = du; lastDD = dd;

        MatchPattern.tryDetect();
        // IMPORTANT Must swing the turret to the intake position before attempting to calibrate.
        // Hold X to calibrate turret to current position as parked
        if (gamepad1.cross) {
            Turret.INSTANCE.calibrateToParkedPosition();
            gamepad1.rumbleBlips(2); // confirms it took
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pattern", MatchPattern.getPattern());
        telemetry.addLine("Hold ✕ with turret facing FRONT to recalibrate");
        telemetry.addData("Turret Angle", Turret.INSTANCE.getTurretAngle()); // live feedback
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // START — scan slots, set pose, pick initial mode
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void start() {
        MatchPattern.reset();
        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);

        // IMPORTANT: Set initial field position based on selected alliance.
        Pinpoint.INSTANCE.updatePosition(alliance == Aliance.BLUE
                ? new Pose2D(DistanceUnit.INCH, 135.5, 9, AngleUnit.DEGREES, 0)
                : new Pose2D(DistanceUnit.INCH, 8.5,   9, AngleUnit.DEGREES, 180));

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

        // ── TRIANGLE: intake toggle / rescan ──────────────────────────────────
        if (triangle && !lastTriangle) {
            if (mode == RobotMode.SHOOT) {
                // Switch back to intake
                enterIntakeMode();
            } else if (!intakeRunning) {
                // Start intake
                intakeRunning = true;
                Intake.INSTANCE.on();
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                // Intake already running — rescan all slots then resume
                intakeRunning = false;
                dwelling = false;
                Intake.INSTANCE.idle();
                rescanIndex = 0;
                rescanState = RescanState.MOVING;
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[0]);
                rescanTimer.reset();
            }
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
        if (cross && !lastCross
                && mode == RobotMode.SHOOT
                && flywheelState == 2
                && shootState == ShootState.IDLE
                && flickState == FlickState.IDLE) {

            shootTimer.reset();
            shootState = ShootState.CONFIRM;
            triggerFlick();
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
            Turret.INSTANCE.zeroAngleOffset(); // offset only, no odometry change
        }

        if (dd && !lastDD) {
            double[] visionPose = Limelight.INSTANCE.getAveragedSnapshotPose(5);
            if (visionPose != null) {
                double visionX = visionPose[0];
                double visionY = visionPose[1];
                if (visionX > 1.0 && visionX < 143.0 && visionY > 1.0 && visionY < 143.0) {
                    Pinpoint.INSTANCE.relocalizePositionFromTag(visionX, visionY);
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

        // ── Left trigger: reverse intake ──────────────────────────────────────
        if (lt && !lastLT) {
            if (intakeRunning) Intake.INSTANCE.reverse();
            else               Intake.INSTANCE.idle();
        }

        if (rt && !lastRT) {
            turretLock = !turretLock;
            gamepad1.rumbleBlips(turretLock ? 3 : 1);
        }

        // ── Bumpers: manual spindexer step ────────────────────────────────────
        if (lb && !lastLB) { Spindexer.Position.next();     Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()); }
        if (rb && !lastRB) { Spindexer.Position.previous(); Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[rb ? 1 : 0]); } // Fix typo in original

        // ─────────────────────────────────────────────────────────────────────
        // STATE MACHINES
        // ─────────────────────────────────────────────────────────────────────

        // ── Rescan ────────────────────────────────────────────────────────────
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
        if (mode == RobotMode.INTAKE
                && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && flickState == FlickState.IDLE) {

            if (intakeRunning) {
                // Only read/dwell if current slot is actually empty
                if (Spindexer.INSTANCE.getBallAtPosition()[
                        Spindexer.INSTANCE.getPosition().ordinal()] != Spindexer.DetectedColor.EMPTY) {
                    // Slot already has a ball — find next free one
                    dwelling = false;
                    int free = Spindexer.INSTANCE.freePosition();
                    if (free != -1)
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                } else if (!dwelling) {
                    dwelling = true;
                    dwellTimer.reset();
                } else if (dwellTimer.seconds() >= INTAKE_DWELL_SEC) {
                    Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                    dwelling = false;
                    Spindexer.INSTANCE.periodic();

                    if (Spindexer.INSTANCE.getFull()) {
                        enterShootMode();
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1)
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                }
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
            if (flickState == FlickState.IDLE) {
                if (Spindexer.INSTANCE.getEmpty()) {
                    enterIntakeMode();
                } else {
                    int next = nextShootSlot();
                    if (next != -1)
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                }
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
            // Now pull the snapshot with the freshly updated orientation alignment
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
    // TELEMETRY
    // ─────────────────────────────────────────────────────────────────────

            double llDist = Limelight.INSTANCE.distanceFromTag(goalId);

            Spindexer.DetectedColor[] balls = Spindexer.INSTANCE.getBallAtPosition();

            int ballCount = 0;
            for (Spindexer.DetectedColor c : balls)
                if (c != Spindexer.DetectedColor.EMPTY) ballCount++;

            String flywheelStatus =
                    flywheelState == 0 ? "OFF" :
                            flywheelState == 1 ? "SPINNING" :
                                    "READY";

            boolean readyToShoot =
                    flywheelState == 2 &&
                            Turret.INSTANCE.isSettled();

            telemetry.addLine("════════ MATCH STATUS ════════");
            telemetry.addData("Mode", mode);
            telemetry.addData("Pattern",
                    MatchPattern.getPattern() +
                            (MatchPattern.isLocked() ? " 🔒" : ""));
            telemetry.addData("Ready", readyToShoot ? "✓ YES" : "NO");

            telemetry.addLine();

            telemetry.addLine("════════ SHOOTER ═════════════");
            telemetry.addData("Flywheel", flywheelStatus);
            telemetry.addData("RPM",
                    String.format("%d / %d",
                            (int) Turret.INSTANCE.getVelocity(),
                            (int) Turret.INSTANCE.distanceToVelocity(px, py, alliance)));
            telemetry.addData("AutoShoot", shootState);

            telemetry.addLine();

            telemetry.addLine("════════ BALLS ═══════════════");
            telemetry.addData("Count", ballCount + "/3");
            telemetry.addData("Slot 1", balls[0]);
            telemetry.addData("Slot 2", balls[1]);
            telemetry.addData("Slot 3", balls[2]);

            telemetry.addLine();

            telemetry.addLine("════════ ROBOT ═══════════════");
            telemetry.addData("Distance",
                    llDist > 0 ? String.format("%.1f in", llDist) : "NO TARGET");
            telemetry.addData("Heading",
                    String.format("%.1f°", Pinpoint.INSTANCE.getHeading()));
            telemetry.addData("Pos",
                    String.format("(%.0f, %.0f)", px, py));
        double[] raw = Limelight.INSTANCE.getMegaTagPoseRaw();
        double[] converted = Limelight.INSTANCE.getMegaTagPose();
        if (raw != null) {
            telemetry.addData("MT2 raw inches", String.format("(%.1f, %.1f)", raw[0], raw[1]));
        }
        if (converted != null) {
            telemetry.addData("MT2 converted", String.format("(%.1f, %.1f)", converted[0], converted[1]));
        }

        telemetry.addData("Turret Target", Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Turret Actual", Turret.INSTANCE.getTurretAngle());
        telemetry.addData("TurretOffset", Turret.INSTANCE.getTurretOffSet());

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
     */
    /**
     * IMPORTANT: Cycles through all spindexer slots and updates the internal color map.
     * Automatically transitions to shooting mode if full, or resumes intaking if slots remain.
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
                    // Update spindexer state with the newly scanned colors
                    Spindexer.INSTANCE.periodic();

                    if (Spindexer.INSTANCE.getFull()) {
                        // SCENARIO A: Spindexer is completely full.
                        // Automatically switch to shoot mode, spin up flywheels, and target the first ball.
                        enterShootMode();
                    } else {
                        // SCENARIO B: Spindexer has empty slots.
                        // Automatically find the next free slot and resume intaking.
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) {
                            mode = RobotMode.INTAKE;
                            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);

                            // Turn the intake system back on
                            intakeRunning = true;
                            Intake.INSTANCE.on();

                            // IMPORTANT: Reset dwelling so the dwell-timer starts fresh for the new slot
                            dwelling = false;
                        } else {
                            // Fallback safety
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

                    // IMPORTANT: Position type toggling ensures the spindexer centers correctly for the next ball.
                    int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    Spindexer.INSTANCE.periodic();

                    flickTimer.reset();
                    flickState   = FlickState.IDLE;
                    transferDown = true;

                    // Spin back up for the next ball.
                    flywheelState = 1;
                    rumbled       = false;
                }
                break;
            default: break;
        }
    }

    /**
     * IMPORTANT: Automates the sequence of shooting multiple balls.
     * Uses the distance sensor to confirm when a ball has cleared the shooter.
     */
    private void tickAutoShoot() {
        switch (shootState) {
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
                        && Turret.INSTANCE.isSettled()
                        && flickState == FlickState.IDLE
                        && transferDown) {
                    shootTimer.reset();
                    shootState = ShootState.CONFIRM;
                    triggerFlick();
                }
                break;

            case NEXT_BALL:
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootState = ShootState.IDLE;
                    break;
                }
                // Fire next ball once turret and flywheels have recovered.
                if (Turret.INSTANCE.isSettled() && flickState == FlickState.IDLE && transferDown) {
                    shootTimer.reset();
                    shootState = ShootState.CONFIRM;
                    triggerFlick();
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
        Intake.INSTANCE.idle();
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        int next = nextShootSlot();
        if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
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
        if (flickState == FlickState.IDLE && transferDown) {
            transferDown  = false;
            Transfer.INSTANCE.transferUpAggressive(); // TODO: Remove this after Banquet
            flickTimer.reset();
            flickState    = FlickState.WAIT_UP;
            flywheelState = 1;   // reset ready state — re-confirm speed after flick.
            rumbled       = false;
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
            if (order != null) {
                int shotCount = 0;
                for (Spindexer.DetectedColor c : slots)
                    if (c == Spindexer.DetectedColor.EMPTY) shotCount++;
                if (shotCount < order.length) {
                    Spindexer.DetectedColor needed = order[shotCount];
                    for (int i = 0; i < slots.length; i++)
                        if (slots[i] == needed) return i;
                }
            }
        }

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
