package org.firstinspires.ftc.teamcode.tuning;

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

@Config
@TeleOp(name = "DataCollection")
public class DataCollection extends OpMode {

    // ── Dashboard-tunable ─────────────────────────────────────────────────────
    public static double intakeDwellSec = 0.3;

    // ── Hardcoded alliance ────────────────────────────────────────────────────
    private final Aliance currentAliance = Aliance.BLUE;

    private static final double START_X   = 8.5;
    private static final double START_Y   = 8.875;
    private static final double START_HDG = 90;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false;
    private boolean intakeOn     = false;
    private boolean transferDown = true;

    // ── Transfer flick state machine — no jam retry ───────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Spindexer intake dwell ────────────────────────────────────────────────
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;

    private double targetVelocity = 0;
    private double targetHood     = 1.0;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastDpadDown;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Intake.INSTANCE.init(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        telemetry.addLine("Alliance: BLUE (hardcoded)");
        telemetry.update();
    }

    @Override
    public void start() { resetOdometry(); }

    @Override
    public void loop() {
        boolean circle       = gamepad1.circle;
        boolean cross        = gamepad1.cross;
        boolean square       = gamepad1.square;
        boolean triangle     = gamepad1.triangle;
        boolean leftBumper   = gamepad1.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper;
        boolean dpadLeft     = gamepad1.dpad_left;
        boolean dpadDown     = gamepad1.dpad_down;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;

        if (triangle && !lastTriangle) {
            intakeOn  = true;
            shootMode = false;
            dwelling  = false;
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        if (circle && !lastCircle) {
            if (intakeOn) {
                intakeOn = false;
                dwelling = false;
                Intake.INSTANCE.idle();
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

        if (square && !lastSquare) resetOdometry();
        if (dpadDown && !lastDpadDown) Turret.INSTANCE.zeroAngleOffset();
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

        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x);

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
            Spindexer.INSTANCE.periodic();
            if (transferDown) {
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootMode = false;
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                } else {
                    int filled = Spindexer.INSTANCE.filledPosition();
                    if (filled != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                }
            }
        }

        double px = Pinpoint.INSTANCE.getPosX();
        double py = Pinpoint.INSTANCE.getPosY();

        if (!shootMode) {
            targetVelocity = 500;
            targetHood     = 1.0;
            Turret.INSTANCE.setToAngle(270);
        } else {
            targetVelocity = Turret.INSTANCE.distanceToVelocity(px, py, currentAliance);
            targetHood     = Turret.INSTANCE.distanceToPosition(px, py, currentAliance);
            Turret.INSTANCE.followGoalOdometryPositional(currentAliance);
        }

        Turret.INSTANCE.setVelocity(targetVelocity);
        Turret.INSTANCE.setHoodPosition(targetHood);

        Spindexer.DetectedColor detected = intakeOn
                ? Spindexer.INSTANCE.readCurrentColor() : Spindexer.DetectedColor.EMPTY;
        if      (detected == Spindexer.DetectedColor.GREEN)  gamepad1.rumble(1.0, 0.0, 200);
        else if (detected == Spindexer.DetectedColor.PURPLE) gamepad1.rumble(0.0, 1.0, 200);

        Pinpoint.INSTANCE.periodic();
        Turret.INSTANCE.periodic();

        telemetry.addLine("── MATCH ────────────────────────────────");
        telemetry.addData("Alliance", "BLUE (hardcoded)");
        telemetry.addData("Mode",     shootMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Intake",   intakeOn  ? "ON"    : "OFF");
        telemetry.addLine("── ODOMETRY ─────────────────────────────");
        telemetry.addData("x",       px);
        telemetry.addData("y",       py);
        telemetry.addData("Heading", Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addLine("── TURRET ───────────────────────────────");
        telemetry.addData("Turret Angle Set",  Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Turret Power Set",  Turret.INSTANCE.getTurretPowerSet());
        telemetry.addData("Velocity (actual)", (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)", (int) targetVelocity);
        telemetry.addData("Hood (target)",     targetHood);
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
        telemetry.addData("Empty",            Spindexer.INSTANCE.getEmpty());
        telemetry.addData("Full",             Spindexer.INSTANCE.getFull());
        telemetry.update();

        lastCircle      = circle;
        lastCross       = cross;
        lastSquare      = square;
        lastTriangle    = triangle;
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadDown    = dpadDown;
    }

    private void resetOdometry() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HDG));
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
    }
}