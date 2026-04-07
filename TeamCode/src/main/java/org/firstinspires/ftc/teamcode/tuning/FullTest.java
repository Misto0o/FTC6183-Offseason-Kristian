package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

@Config
@TeleOp(name = "FullTest", group = "FullSystem")
public class FullTest extends OpMode {

    // ── Dashboard-tunable ─────────────────────────────────────────────────────
    public static double hoodPosition   = 1.0;
    public static double shootVelocity  = 3000;
    public static double intakeDwellSec = 0.3;

    // ── Hardcoded alliance ────────────────────────────────────────────────────
    private final Aliance currentAliance = Aliance.BLUE;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false;
    private boolean intakeOn     = false;
    private boolean transferDown = true;

    // ── Square flywheel state machine ─────────────────────────────────────────
    private int     squareState = 0;
    private boolean hasRumbled  = false;

    // ── Transfer flick state machine ──────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Spindexer intake dwell ────────────────────────────────────────────────
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;

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
        Limelight.INSTANCE.initialize(hardwareMap);
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        telemetry.addLine("Alliance: BLUE (hardcoded)");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean circle       = gamepad1.circle;
        boolean cross        = gamepad1.cross;
        boolean square       = gamepad1.square;
        boolean triangle     = gamepad1.triangle;
        boolean leftBumper   = gamepad1.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;
        boolean dpadLeft     = gamepad1.dpad_left;

        if (triangle && !lastTriangle) {
            intakeOn    = true;
            shootMode   = false;
            squareState = 0;
            hasRumbled  = false;
            dwelling    = false;
            Turret.INSTANCE.setVelocity(0);
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        if (circle && !lastCircle) {
            if (intakeOn) {
                intakeOn = false;
                dwelling = false;
                Intake.INSTANCE.idle();
                Turret.INSTANCE.setVelocity(0);
                Limelight.INSTANCE.stop();
                Turret.INSTANCE.resetFineTune();
            } else {
                shootMode = true;
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            }
        }

        if (cross && !lastCross && flickState == FlickState.IDLE && transferDown) {
            transferDown = false;
            Transfer.INSTANCE.transferUp();
            flickTimer.reset();
            flickState = FlickState.WAIT_UP;
        }

        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    hasRumbled  = false;
                    squareState = 1;
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

        if (dpadLeft) {
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }
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

        // ── Transfer flick state machine — no jam retry ───────────────────────
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
                    Spindexer.INSTANCE.setColor(
                            Spindexer.INSTANCE.getPosition(),
                            Spindexer.DetectedColor.EMPTY);
                    transferDown = true;
                    flickState   = FlickState.IDLE;
                }
                break;
            default:
                break;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // ── Intake dwell logic ────────────────────────────────────────────────
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
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
                        shootMode = true;
                        intakeOn  = false;
                        Intake.INSTANCE.idle();
                        Intake.INSTANCE.idle();
                        Limelight.INSTANCE.start();
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                }
            } else {
                dwelling = false;
            }
        } else {
            dwelling = false;
            if (flickState == FlickState.IDLE && transferDown) {
                Spindexer.INSTANCE.periodic();
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootMode   = false;
                    squareState = 0;
                    Turret.INSTANCE.setVelocity(0);
                    Limelight.INSTANCE.stop();
                    Turret.INSTANCE.resetFineTune();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                } else {
                    int filled = Spindexer.INSTANCE.filledPosition();
                    if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                }
            }
        }

        int goalId = Limelight.BLUE_GOAL_ID;
        double targetVelocity;

        if (!shootMode) {
            targetVelocity = 0;
            Turret.INSTANCE.setHoodPosition(1.0);
        } else {
            double llDist = Limelight.INSTANCE.distanceFromTag(goalId);
            if (llDist > 0) {
                targetVelocity = Turret.INSTANCE.distanceToVelocity(llDist, 0, currentAliance);
                Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(llDist, 0, currentAliance));
            } else {
                targetVelocity = shootVelocity;
                Turret.INSTANCE.setHoodPosition(hoodPosition);
            }
            if (shootVelocity > 0 && shootVelocity < targetVelocity) targetVelocity = shootVelocity;
        }

        if (shootMode) Turret.INSTANCE.aimAtGoal(currentAliance, goalId);
        if (squareState > 0) Turret.INSTANCE.setVelocity(targetVelocity);
        else                  Turret.INSTANCE.setVelocity(0);

        if (squareState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - targetVelocity) < Turret.threshold
                && !hasRumbled) {
            gamepad1.rumbleBlips(1);
            hasRumbled  = true;
            squareState = 2;
        }

        Spindexer.DetectedColor detected = intakeOn
                ? Spindexer.INSTANCE.readCurrentColor() : Spindexer.DetectedColor.EMPTY;
        if      (detected == Spindexer.DetectedColor.GREEN)  gamepad1.rumble(1.0, 0.0, 200);
        else if (detected == Spindexer.DetectedColor.PURPLE) gamepad1.rumble(0.0, 1.0, 200);

        Turret.INSTANCE.periodic();

        double llDistanceTelem = Limelight.INSTANCE.distanceFromTag(goalId);
        telemetry.addLine("── MATCH ────────────────────────────────");
        telemetry.addData("Alliance", "BLUE (hardcoded)");
        telemetry.addData("Mode",     shootMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Intake",   intakeOn  ? "ON"    : "OFF");
        telemetry.addLine("── FLYWHEEL ─────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" : squareState == 1 ? "SPINNING UP..." : "READY - Press Square");
        telemetry.addData("Velocity (actual)", (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)", (int) targetVelocity);
        telemetry.addData("Hood Position",     hoodPosition);
        telemetry.addLine("── LIMELIGHT ────────────────────────────");
        telemetry.addData("LL Distance", llDistanceTelem > 0 ? llDistanceTelem + " in" : "NOT FOUND");
        telemetry.addData("Pattern",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP" :
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP" :
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG" : "Not found");
        telemetry.addLine("── TURRET ───────────────────────────────");
        telemetry.addData("Turret Angle",     Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Turret Angle Set", Turret.INSTANCE.getTurretAngleSet());
        telemetry.addLine("── SPINDEXER ────────────────────────────");
        telemetry.addData("Transfer Down",    transferDown);
        telemetry.addData("Flick State",      flickState);
        telemetry.addData("Dwell Active",     dwelling);
        telemetry.addData("Detected Color",   detected);
        telemetry.addData("Ball Pos 1",       Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball Pos 2",       Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball Pos 3",       Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Current Position", Spindexer.INSTANCE.getPosition());
        telemetry.addData("Free Position",    Spindexer.INSTANCE.freePosition());
        telemetry.update();

        lastCircle      = circle;
        lastCross       = cross;
        lastSquare      = square;
        lastTriangle    = triangle;
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        Limelight.INSTANCE.stop();
    }
}