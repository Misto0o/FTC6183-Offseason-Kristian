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

@Config
@TeleOp(name = "Teleop", group = "!")
public class Teleop extends OpMode {

    // ── Dashboard-tunable ─────────────────────────────────────────────────────
    public static double shootVelocity  = 3000;
    public static double turretAngle    = 123.5;
    public static double intakeDwellSec = 0.3;
    public static double hoodIncrement  = 0.05;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private Aliance currentAliance = Aliance.BLUE;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false;
    private boolean turretLock   = false;
    private boolean transferDown = true;
    private boolean intakeOn     = false;
    private boolean intakeDwellMode = false;

    // ── Flywheel state: 0=off  1=spinning up  2=ready ─────────────────────────
    private int     squareState = 0;
    private boolean hasRumbled  = false;
    private int     patternStep = 0;

    // ── Hood manual override ──────────────────────────────────────────────────
    private double hoodOverride = -1;

    // ── Transfer flick state machine ──────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Intake dwell ──────────────────────────────────────────────────────────
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
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
        telemetry.update();
    }

    @Override
    public void start() {
        Turret.INSTANCE.setToAngle(turretAngle);
        if (currentAliance == Aliance.BLUE) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0));
        } else {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 8.5, 9, AngleUnit.DEGREES, 180));
        }

        // Scan all slots — same as Outreach dwell pattern
        for (int i = 0; i < Spindexer.Position.values().length; i++) {
            Spindexer.Position pos = Spindexer.Position.values()[i];
            Spindexer.INSTANCE.setToPosition(pos);
            try { Thread.sleep(400); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            Spindexer.INSTANCE.setColor(pos, Spindexer.INSTANCE.readCurrentColor());
        }
        Spindexer.INSTANCE.periodic();

        if (Spindexer.INSTANCE.getFull()) {
            // Full — go to shoot, start revving, squareState=1 so driver waits for rumble
            shootMode   = true;
            squareState = 1;
            hasRumbled  = false;
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            int filled = Spindexer.INSTANCE.filledPosition();
            if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
        } else {
            shootMode       = false;
            intakeOn        = false;
            intakeDwellMode = false;
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

        // ── Triangle: intake on / dwell toggle ────────────────────────────────
        if (triangle && !lastTriangle) {
            if (!intakeOn) {
                intakeOn        = true;
                intakeDwellMode = false;
                shootMode       = false;
                squareState     = 0;
                hasRumbled      = false;
                patternStep     = 0;
                dwelling        = false;
                hoodOverride    = -1;
                Turret.INSTANCE.setVelocity(0);
                Turret.INSTANCE.setToAngle(turretAngle);
                Intake.INSTANCE.on();
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                // Second press: toggle dwell mode (re-run for false-full fix)
                intakeDwellMode = !intakeDwellMode;
                dwelling        = false;
                dwellTimer.reset();
            }
        }

        // ── Circle: kill everything ───────────────────────────────────────────
        if (circle && !lastCircle) {
            intakeOn        = false;
            intakeDwellMode = false;
            dwelling        = false;
            shootMode       = false;
            squareState     = 0;
            hasRumbled      = false;
            patternStep     = 0;
            flickState      = FlickState.IDLE;
            transferDown    = true;
            hoodOverride    = -1;
            Intake.INSTANCE.idle();
            Turret.INSTANCE.setVelocity(0);
            Transfer.INSTANCE.transferDown();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        // ── Cross: hood increment ─────────────────────────────────────────────
        if (cross && !lastCross) {
            if (hoodOverride < 0) hoodOverride = Turret.INSTANCE.getPosition();
            hoodOverride += hoodIncrement;
            if (hoodOverride > 1.0) hoodOverride = -1;
        }

        // ── Square: flywheel cycle ────────────────────────────────────────────
        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    squareState = 1;
                    hasRumbled  = false;
                    patternStep = 0;
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
                    patternStep = 0;
                    break;
            }
        }

        if (dpadUp    && !lastDpadUp)    turretLock = !turretLock;
        if (dpadDown  && !lastDpadDown)  Turret.INSTANCE.zeroAngleOffset();
        if (dpadLeft  && !lastDpadLeft) {
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }
        if (dpadRight && !lastDpadRight) hoodOverride = -1;
        if (rightTrigger) Turret.INSTANCE.updateAngleOffset(-0.1);
        if (leftTrigger)  Turret.INSTANCE.updateAngleOffset(0.1);

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
                    // Swing to intake pos, read sensor, stamp whatever we see
                    int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
                    // Small settle then read — reuse flickTimer for another 0.3s
                    flickTimer.reset();
                    flickState = FlickState.IDLE; // handled below after settle
                    transferDown = true;

                    // Stamp what sensors see after brief settle inline with next loop ticks
                    // We use a simple approach: mark empty now, sensors will correct on next intake dwell
                    Spindexer.INSTANCE.setColor(
                            Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);

                    // Advance pattern step
                    if (shootMode && MatchPattern.isLocked()) {
                        patternStep = (patternStep + 1) % 3;
                    }
                    Spindexer.INSTANCE.periodic();

                    if (Spindexer.INSTANCE.getEmpty()) {
                        shootMode   = false;
                        squareState = 0;
                        patternStep = 0;
                        Turret.INSTANCE.setVelocity(0);
                        Turret.INSTANCE.setToAngle(turretAngle);
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    } else {
                        int targetPos = -1;
                        for (int offset = 0; offset < 3; offset++) {
                            int step = (patternStep + offset) % 3;
                            Spindexer.DetectedColor c = getTargetColor(step);
                            int pos = findColorPosition(c);
                            if (pos != -1
                                    && Spindexer.INSTANCE.getBallAtPosition()[pos] != Spindexer.DetectedColor.EMPTY) {
                                targetPos   = pos;
                                patternStep = step;
                                break;
                            }
                        }
                        if (targetPos != -1) {
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[targetPos]);
                        } else {
                            int filled = Spindexer.INSTANCE.filledPosition();
                            if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                        }
                    }
                }
                break;

            default:
                break;
        }

        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x);

        // ── INTAKE LOGIC — straight from Outreach, no changes ─────────────────
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && flickState == FlickState.IDLE) {
            if (intakeOn) {
                Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
                if (!dwelling) {
                    dwelling = true;
                    dwellTimer.reset();
                } else if (dwellTimer.seconds() >= intakeDwellSec) {
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                    dwelling = false;
                    Spindexer.INSTANCE.periodic();
                    if (Spindexer.INSTANCE.getFull()) {
                        // Full — switch to shoot, start revving
                        shootMode       = true;
                        intakeOn        = false;
                        intakeDwellMode = false;
                        Intake.INSTANCE.idle();
                        squareState = 1;
                        hasRumbled  = false;
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                        int filled = Spindexer.INSTANCE.filledPosition();
                        if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
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
            // In shoot mode and not mid-flick — position to next target
            if (Spindexer.INSTANCE.getEmpty()) {
                shootMode   = false;
                squareState = 0;
                patternStep = 0;
                Turret.INSTANCE.setVelocity(0);
                Turret.INSTANCE.setToAngle(turretAngle);
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                int targetPos = -1;
                for (int offset = 0; offset < 3; offset++) {
                    int step = (patternStep + offset) % 3;
                    Spindexer.DetectedColor c = getTargetColor(step);
                    int pos = findColorPosition(c);
                    if (pos != -1
                            && Spindexer.INSTANCE.getBallAtPosition()[pos] != Spindexer.DetectedColor.EMPTY) {
                        targetPos   = pos;
                        patternStep = step;
                        break;
                    }
                }
                if (targetPos != -1) {
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[targetPos]);
                } else {
                    int filled = Spindexer.INSTANCE.filledPosition();
                    if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                }
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
            if (shootVelocity > 0 && shootVelocity < targetVelocity) targetVelocity = shootVelocity;
        }
        if (hoodOverride >= 0) targetHood = hoodOverride;

        if (!shootMode || turretLock) Turret.INSTANCE.setToAngle(turretAngle);
        else                          Turret.INSTANCE.followGoalOdometryPositional(currentAliance);

        if (squareState > 0) Turret.INSTANCE.setVelocity(targetVelocity);
        else                  Turret.INSTANCE.setVelocity(0);
        Turret.INSTANCE.setHoodPosition(targetHood);

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
        telemetry.addData("Intake",   !intakeOn ? "OFF" : intakeDwellMode ? "ON (DWELL)" : "ON");

        telemetry.addLine("── FLYWHEEL ─────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP..." : "READY - Press Square");
        telemetry.addData("Velocity (actual)",    (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)",    (int) targetVelocity);
        telemetry.addData("Next Expected",        MatchPattern.isLocked()
                ? String.valueOf(getTargetColor(patternStep)) : "ANY");

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
    }

    private Spindexer.DetectedColor getTargetColor(int step) {
        if (!MatchPattern.isLocked()) return null;
        step = step % 3;
        switch (MatchPattern.getPattern()) {
            case GPP: return step == 0 ? Spindexer.DetectedColor.GREEN  : Spindexer.DetectedColor.PURPLE;
            case PGP: return step == 1 ? Spindexer.DetectedColor.GREEN  : Spindexer.DetectedColor.PURPLE;
            case PPG: return step == 2 ? Spindexer.DetectedColor.GREEN  : Spindexer.DetectedColor.PURPLE;
            default:  return null;
        }
    }

    private int findColorPosition(Spindexer.DetectedColor target) {
        if (target == null || target == Spindexer.DetectedColor.EMPTY) return -1;
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == target) return i;
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