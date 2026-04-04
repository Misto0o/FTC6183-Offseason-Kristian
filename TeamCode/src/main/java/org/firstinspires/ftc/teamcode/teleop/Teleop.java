package org.firstinspires.ftc.teamcode.teleop;

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
import java.util.Locale;

@Config
@TeleOp(name = "Teleop", group = "MAIN")
public class Teleop extends OpMode {

    // ── Dashboard-tunable ─────────────────────────────────────────────────────
    public static double shootVelocity  = 3000;
    public static double turretAngle    = 123.5;
    public static double intakeDwellSec = 0.3;
    public static  double SHOT_CONFIRM_TIMEOUT = 1.5;
    public static  double REFIRE_DELAY         = 0.5;

    public static double hoodIncrement = 0.5;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private Aliance currentAliance = Aliance.BLUE;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false;
    private boolean turretLock   = false;
    private boolean transferDown = true;
    private boolean intakeOn     = false;

    // ── Flywheel state: 0=off  1=spinning up  2=ready ─────────────────────────
    private int     squareState = 0;
    private boolean hasRumbled  = false;

    // ── Distance sensor ───────────────────────────────────────────────────────
    private final DistanceSensor distanceSensor = new DistanceSensor();

    // ── Auto-shoot state machine ──────────────────────────────────────────────
    private enum AutoShootState { IDLE, CONFIRM_SHOT, WAIT_REFIRE, NEXT_BALL }
    private AutoShootState autoShootState = AutoShootState.IDLE;
    private final ElapsedTime autoShootTimer = new ElapsedTime();

    // ── Hood manual override ──────────────────────────────────────────────────
    private double hoodOverride = -1;

    // ── Transfer flick state machine ──────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Intake dwell ──────────────────────────────────────────────────────────
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;
    private enum RescanState { IDLE, MOVING, READING }
    private RescanState rescanState = RescanState.IDLE;
    private int rescanIndex = 0;
    private final ElapsedTime rescanTimer = new ElapsedTime();

    private final int[] intakeOrder = new int[3]; // stores slot index in order of intake
    private int intakeCount = 0;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastLeftTrigger, lastRightTrigger;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        distanceSensor.init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDown();
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
        MatchPattern.reset();
        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up   && !lastDpadUp)   currentAliance = Aliance.BLUE;
        if (gamepad1.dpad_down && !lastDpadDown)  currentAliance = Aliance.RED;
        lastDpadUp   = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        MatchPattern.tryDetect();
        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.addData("Alliance", currentAliance);
        telemetry.addData("Pattern",  MatchPattern.getPattern());
        telemetry.addData("Sensor found", hardwareMap.analogInput.contains("distanceSensor") ? "YES" : "NO - CHECK CONFIG NAME");
        telemetry.update();
    }

    @Override
    public void start() {
        MatchPattern.reset();
        Turret.INSTANCE.setToAngle(turretAngle);
        if (currentAliance == Aliance.BLUE) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0));
        } else {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 8.5, 9, AngleUnit.DEGREES, 180));
        }

        // Scan all slots — blocking is acceptable here, runs once before match starts
        for (int i = 0; i < Spindexer.Position.values().length; i++) {
            Spindexer.Position pos = Spindexer.Position.values()[i];
            Spindexer.INSTANCE.setToPosition(pos);
            //noinspection BusyWait
            try { Thread.sleep(400); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            Spindexer.INSTANCE.setColor(pos, Spindexer.INSTANCE.readCurrentColor());
        }
        Spindexer.INSTANCE.periodic();

        if (Spindexer.INSTANCE.getFull()) {
            shootMode       = true;
            intakeOn        = false;
            squareState     = 1;
            hasRumbled      = false;
            Intake.INSTANCE.idle();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            int next = nextShootPosition();
            if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
        } else {
            shootMode   = false;
            intakeOn    = false;
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
        }
    }

    @Override
    public void loop() {
        if (!MatchPattern.isLocked()) MatchPattern.tryDetect();

        boolean circle       = gamepad1.circle;
        boolean cross        = gamepad1.cross;
        boolean square       = gamepad1.square;
        boolean triangle     = gamepad1.triangle;
        boolean leftBumper   = gamepad1.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper;
        boolean dpadUp       = gamepad1.dpad_up;
        boolean dpadDown     = gamepad1.dpad_down;
        boolean dpadLeft     = gamepad1.dpad_left;
        boolean dpadRight    = gamepad1.dpad_right;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;
        // ── Triangle: intake on / rescan toggle ───────────────────────────────
        if (triangle && !lastTriangle) {
            if (!intakeOn) {
                intakeOn        = true;
                shootMode       = false;
                squareState     = 0;
                hasRumbled      = false;
                dwelling        = false;
                hoodOverride    = -1;
                intakeCount     = 0;
                Turret.INSTANCE.setVelocity(0);
                Turret.INSTANCE.setToAngle(turretAngle);
                Intake.INSTANCE.on();
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                // Second press — kick off non-blocking rescan
                Intake.INSTANCE.idle();
                intakeOn    = false;
                dwelling    = false;
                intakeCount = 0;
                rescanIndex = 0;
                rescanState = RescanState.MOVING;
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[0]);
                rescanTimer.reset();
            }
        }
        switch (rescanState) {
            case MOVING:
                if (rescanTimer.seconds() >= 0.15) {
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
                    int free = Spindexer.INSTANCE.freePosition();
                    if (free != -1) {
                        intakeOn = true;
                        Intake.INSTANCE.on();
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                    rescanState = RescanState.IDLE;
                } else {
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[rescanIndex]);
                    rescanTimer.reset();
                    rescanState = RescanState.MOVING;
                }
                break;
            default:
                break;
        }

        // ── Circle: kill everything ───────────────────────────────────────────
        if (circle && !lastCircle) {
            intakeOn        = false;
            dwelling        = false;
            shootMode       = false;
            squareState     = 0;
            hasRumbled      = false;
            flickState      = FlickState.IDLE;
            transferDown    = true;
            hoodOverride    = -1;
            autoShootState  = AutoShootState.IDLE;
            Intake.INSTANCE.idle();
            Turret.INSTANCE.setVelocity(0);
            Transfer.INSTANCE.transferDown();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        // ── Cross: start auto-shoot-all / hood increment ──────────────────────
        if (cross && !lastCross) {
            if (shootMode && squareState == 2 && autoShootState == AutoShootState.IDLE) {
                autoShootState = AutoShootState.CONFIRM_SHOT;
                autoShootTimer.reset();
                if (flickState == FlickState.IDLE && transferDown) {
                    transferDown = false;
                    Transfer.INSTANCE.transferUp();
                    flickTimer.reset();
                    flickState  = FlickState.WAIT_UP;
                    squareState = 1;
                    hasRumbled  = false;
                }
            } else {
                if (hoodOverride < 0) hoodOverride = Turret.INSTANCE.getPosition();
                hoodOverride += hoodIncrement;
                if (hoodOverride > 1.0) hoodOverride = -1;
            }
        }

        // ── Square: flywheel cycle ────────────────────────────────────────────
        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    squareState = 1;
                    hasRumbled  = false;
                    break;
                case 2:
                    if (flickState == FlickState.IDLE && transferDown) {
                        transferDown = false;
                        Transfer.INSTANCE.transferUp();
                        flickTimer.reset();
                        flickState  = FlickState.WAIT_UP;
                        squareState = 1;
                        hasRumbled  = false;
                    }
                    break;
                case 1:
                default:
                    Turret.INSTANCE.setVelocity(0);
                    squareState = 0;
                    hasRumbled  = false;
                    break;
            }
        }
        if (dpadUp && !lastDpadUp) {
            // Reset odometry to corner + zero turret offset
            if (currentAliance == Aliance.BLUE) {
                Pinpoint.INSTANCE.updatePosition(
                        new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0));
            } else {
                Pinpoint.INSTANCE.updatePosition(
                        new Pose2D(DistanceUnit.INCH, 8.5, 9, AngleUnit.DEGREES, 180));
            }
            Turret.INSTANCE.zeroAngleOffset();
        }
        if (dpadDown  && !lastDpadDown)  Turret.INSTANCE.zeroAngleOffset();
        if (dpadLeft && !lastDpadLeft) {
            // Force intake mode
            shootMode      = false;
            intakeOn       = true;
            squareState    = 0;
            hasRumbled     = false;
            hoodOverride   = -1;
            autoShootState = AutoShootState.IDLE;
            flickState     = FlickState.IDLE;
            Turret.INSTANCE.setVelocity(0);
            Turret.INSTANCE.setToAngle(turretAngle);
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
        }
        if (dpadRight && !lastDpadRight) {
            // Force shoot mode
            shootMode      = true;
            intakeOn       = false;
            squareState    = 1;
            hasRumbled     = false;
            hoodOverride   = -1;
            autoShootState = AutoShootState.IDLE;
            flickState     = FlickState.IDLE;
            Intake.INSTANCE.idle();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            int next = nextShootPosition();
            if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
        }
        if (leftTrigger && !lastLeftTrigger) {  // hold to reverse intake
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }
        if (rightTrigger && !lastRightTrigger) {  // toggle turret lock
            turretLock = !turretLock;
        }

        if (leftBumper  && !lastLeftBumper) {
            Spindexer.Position.next();
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
        }
        if (rightBumper && !lastRightBumper) {
            Spindexer.Position.previous();
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
        }

        // ── Transfer flick state machine ──────────────────────────────────────
        switch (flickState) {
            case WAIT_UP:
                if (flickTimer.seconds() >= 0.5) {
                    Transfer.INSTANCE.transferDown();
                    flickTimer.reset();
                    flickState = FlickState.WAIT_DOWN;
                }
                break;

            case WAIT_DOWN:
                if (flickTimer.seconds() >= 0.5) {
                    int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
                    flickTimer.reset();
                    flickState   = FlickState.IDLE;
                    transferDown = true;

                    // Mark the shot slot empty
                    Spindexer.INSTANCE.setColor(
                            Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    Spindexer.INSTANCE.periodic();

                    if (Spindexer.INSTANCE.getEmpty()) {
                        shootMode      = false;
                        squareState    = 0;
                        autoShootState = AutoShootState.IDLE;
                        Turret.INSTANCE.setVelocity(0);
                        Turret.INSTANCE.setToAngle(turretAngle);
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    } else {
                        int next = nextShootPosition();
                        if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                    }
                }
                break;

            default:
                break;
        }

        // ── AUTO-SHOOT state machine ──────────────────────────────────────────
        switch (autoShootState) {

            case CONFIRM_SHOT:
                if (flickState != FlickState.IDLE) break;

                if (distanceSensor.isClear()) {
                    autoShootState = AutoShootState.NEXT_BALL;
                } else if (autoShootTimer.seconds() > SHOT_CONFIRM_TIMEOUT) {
                    autoShootTimer.reset();
                    autoShootState = AutoShootState.WAIT_REFIRE;
                }
                break;

            case WAIT_REFIRE:
                if (autoShootTimer.seconds() >= REFIRE_DELAY) {
                    if (squareState == 2 && flickState == FlickState.IDLE && transferDown) {
                        transferDown   = false;
                        Transfer.INSTANCE.transferUp();
                        flickTimer.reset();
                        flickState     = FlickState.WAIT_UP;
                        squareState    = 1;
                        hasRumbled     = false;
                        autoShootTimer.reset();
                        autoShootState = AutoShootState.CONFIRM_SHOT;
                    }
                }
                break;

            case NEXT_BALL:
                if (Spindexer.INSTANCE.getEmpty()) {
                    autoShootState = AutoShootState.IDLE;
                    break;
                }
                if (squareState == 2 && flickState == FlickState.IDLE && transferDown) {
                    transferDown   = false;
                    Transfer.INSTANCE.transferUp();
                    flickTimer.reset();
                    flickState     = FlickState.WAIT_UP;
                    squareState    = 1;
                    hasRumbled     = false;
                    autoShootTimer.reset();
                    autoShootState = AutoShootState.CONFIRM_SHOT;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // ── INTAKE LOGIC ──────────────────────────────────────────────────────
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && flickState == FlickState.IDLE) {
            if (intakeOn) {
                Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
                if (!dwelling) {
                    dwelling = true;
                    dwellTimer.reset();
                } else if (dwellTimer.seconds() >= intakeDwellSec) {
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                    if (intakeCount < intakeOrder.length) {
                        intakeOrder[intakeCount++] = Spindexer.INSTANCE.getPosition().ordinal();
                    }
                    dwelling = false;
                    Spindexer.INSTANCE.periodic();
                    if (Spindexer.INSTANCE.getFull()) {
                        shootMode       = true;
                        intakeOn        = false;
                        Intake.INSTANCE.idle();
                        squareState = 1;
                        hasRumbled  = false;
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                        int next = nextShootPosition();
                        if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                }
            } else {
                dwelling = false;
            }

        } else if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT
                && flickState == FlickState.IDLE) {
            if (Spindexer.INSTANCE.getEmpty()) {
                shootMode   = false;
                squareState = 0;
                Turret.INSTANCE.setVelocity(0);
                Turret.INSTANCE.setToAngle(turretAngle);
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                int next = nextShootPosition();
                if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
            }
        }

        // ── Velocity & hood ───────────────────────────────────────────────────
        int    goalId = (currentAliance == Aliance.BLUE) ? Limelight.BLUE_GOAL_ID : Limelight.RED_GOAL_ID;
        double px     = Pinpoint.INSTANCE.getPosX();
        double py     = Pinpoint.INSTANCE.getPosY();
        double targetVelocity;
        double targetHood;

        if (!shootMode) {
            targetVelocity = 0;
            targetHood     = 1.0;
        } else {
            double llDist = Limelight.INSTANCE.distanceFromTag(goalId);
            if (llDist > 0) {
                targetVelocity = Turret.INSTANCE.distanceToVelocity(llDist, 0, currentAliance);
                targetHood     = Turret.INSTANCE.distanceToPosition(llDist, 0, currentAliance);
            } else {
                targetVelocity = Turret.INSTANCE.distanceToVelocity(px, py, currentAliance);
                targetHood     = Turret.INSTANCE.distanceToPosition(px, py, currentAliance);
            }
            // cap only if dashboard override is set lower than table
            if (shootVelocity > 0 && shootVelocity < targetVelocity) targetVelocity = shootVelocity;
        }
        if (hoodOverride >= 0) targetHood = hoodOverride;

        // Hood ALWAYS updates from table in shoot mode
        Turret.INSTANCE.setHoodPosition(targetHood);

        // Velocity only runs when squareState > 0, but hood always tracks
        if (squareState > 0) Turret.INSTANCE.setVelocity(targetVelocity);
        else                  Turret.INSTANCE.setVelocity(0);

        // ── Rumble: flywheel ready ────────────────────────────────────────────
        if (squareState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - targetVelocity) < Turret.threshold
                && !hasRumbled) {
            gamepad1.rumbleBlips(1);
            hasRumbled  = true;
            squareState = 2;
        }

        // ── Rumble: ball detected during intake ───────────────────────────────
        Spindexer.DetectedColor detected = intakeOn
                ? Spindexer.INSTANCE.readCurrentColor() : Spindexer.DetectedColor.EMPTY;
        if      (detected == Spindexer.DetectedColor.GREEN)  gamepad1.rumble(1.0, 0.0, 200);
        else if (detected == Spindexer.DetectedColor.PURPLE) gamepad1.rumble(0.0, 1.0, 200);

        Turret.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("── MATCH ────────────────────────────────");
        telemetry.addData("Alliance", currentAliance);
        telemetry.addData("Pattern",  MatchPattern.getPattern()
                + (MatchPattern.isLocked() ? " (LOCKED)" : " (searching...)"));
        telemetry.addData("Mode",     shootMode ? "SHOOT" : "INTAKE");

        telemetry.addLine("── FLYWHEEL ─────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP..." : "READY - Press Square");
        telemetry.addData("Velocity (actual)", (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)", (int) targetVelocity);
        telemetry.addData("Next Shot",         nextShootPosition() != -1
                ? Spindexer.INSTANCE.getBallAtPosition()[nextShootPosition()] : "NONE");
        telemetry.addData("Auto-Shoot",        autoShootState);
        telemetry.addData("Distance (cm)",     String.format(Locale.US, "%.1f", distanceSensor.getDistanceCM()));
        telemetry.addData("Ball at shooter",   distanceSensor.isBallPresent() ? "YES" : "no");
        telemetry.addData("px", px);
        telemetry.addData("py", py);
        telemetry.addData("tableVelo", Turret.INSTANCE.distanceToVelocity(px, py, currentAliance));
        telemetry.addData("tableHood", Turret.INSTANCE.distanceToPosition(px, py, currentAliance));
        telemetry.addData("shootMode", shootMode);
        telemetry.addData("LL tx", Limelight.INSTANCE.getTx(goalId));
        telemetry.addData("fineTuneActive", Turret.INSTANCE.fineTuneActive);

        telemetry.addLine("── TURRET ───────────────────────────────");
        telemetry.addData("Turret Lock",   turretLock ? "FORCED" : shootMode ? "TRACKING" : "LOCKED");
        telemetry.addData("Current Angle", Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Angle Set",     Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Hood",          targetHood + (hoodOverride >= 0 ? " (OVERRIDE)" : " (TABLE)"));

        telemetry.addLine("── SPINDEXER ────────────────────────────");
        telemetry.addData("Flick State",      flickState);
        telemetry.addData("Detected",         detected);
        telemetry.addData("Ball Pos 1",       Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball Pos 2",       Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball Pos 3",       Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Pattern locked", MatchPattern.isLocked());
        telemetry.addData("Pattern value",  MatchPattern.getPattern());
        telemetry.addData("nextShoot idx",  nextShootPosition());
        telemetry.addData("Current Position", Spindexer.INSTANCE.getPosition());

        telemetry.update();

        lastCircle      = circle;
        lastCross       = cross;
        lastSquare      = square;
        lastTriangle    = triangle;
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp      = dpadUp;
        lastDpadDown    = dpadDown;
        lastDpadLeft    = dpadLeft;
        lastDpadRight   = dpadRight;
        lastLeftTrigger  = leftTrigger;
        lastRightTrigger = rightTrigger;
    }

    /**
     * Returns the slot index to shoot next based on stored ball colors.
     * If pattern is locked: GREEN first for GPP, PURPLE first for PGP/PPG.
     * If pattern unknown: just returns the first filled slot.
     */
    private int nextShootPosition() {
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();

        if (MatchPattern.isLocked()) {
            switch (MatchPattern.getPattern()) {
                case GPP:
                case PGP:
                case PPG:
                    // Follow intake order — slots were loaded in pattern sequence
                    for (int i = 0; i < intakeCount; i++) {
                        int slot = intakeOrder[i];
                        if (slots[slot] != Spindexer.DetectedColor.EMPTY) return slot;
                    }
                    break;
                default:
                    break;
            }
        }

        // Fallback: any filled slot
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] != Spindexer.DetectedColor.EMPTY) return i;
        }
        return -1;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        Limelight.INSTANCE.stop();
    }
}