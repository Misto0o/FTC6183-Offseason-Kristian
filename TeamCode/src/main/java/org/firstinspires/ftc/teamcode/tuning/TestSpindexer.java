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
import org.firstinspires.ftc.teamcode.Utils.Aliance;

@TeleOp
@Config
public class TestSpindexer extends OpMode {

    public static double spinAngle = 0;
    public static String type      = "";
    public boolean shootCycle      = false;

    // button edge detection
    private boolean prevLeftBumper, prevRightBumper;
    private boolean prevCross, prevTriangle, prevCircle;
    private boolean prevDpadUp, prevDpadLeft, prevDpadRight, prevDpadDown;
    private boolean prevSquare;

    // toggles
    private boolean triangleToggle; // false=shoot, true=intake
    private boolean circleToggle;   // false=idle, true=on
    private boolean dpadUpToggle;

    // shoot-three state machine
    private enum ShootState {
        IDLE,
        SET_POS, WAIT_POS,
        TRANSFER_UP, WAIT_UP,
        TRANSFER_DOWN, WAIT_DOWN,
        MARK_EMPTY, NEXT_BALL
    }
    private ShootState shootState = ShootState.IDLE;
    private int shootBallIndex = 0;
    private static final Spindexer.Position[] SHOOT_ORDER = {
            Spindexer.Position.POSITION_ONE,
            Spindexer.Position.POSITION_TWO,
            Spindexer.Position.POSITION_THREE
    };
    private final ElapsedTime shootTimer = new ElapsedTime();

    // transfer flick state machine (dpadUp)
    private enum FlickState { IDLE, UP, DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain.getInstance().init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);

        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);
        type = "Shoot";
    }

    @Override
    public void loop() {
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean cross       = gamepad1.cross;
        boolean triangle    = gamepad1.triangle;
        boolean circle      = gamepad1.circle;
        boolean dpadUp      = gamepad1.dpad_up;
        boolean dpadLeft    = gamepad1.dpad_left;
        boolean dpadRight   = gamepad1.dpad_right;
        boolean dpadDown    = gamepad1.dpad_down;
        boolean square      = gamepad1.square;

        // Left bumper — next position
        if (leftBumper && !prevLeftBumper)   Spindexer.Position.next();
        // Right bumper — previous position
        if (rightBumper && !prevRightBumper) Spindexer.Position.previous();

        // Cross — start shoot-three sequence
        if (cross && !prevCross && shootState == ShootState.IDLE) {
            shootBallIndex = 0;
            shootCycle     = true;
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            shootState = ShootState.SET_POS;
            shootTimer.reset();
        }

        // Triangle — toggle shoot/intake mode (only if not in shoot cycle)
        if (triangle && !prevTriangle && !shootCycle) {
            triangleToggle = !triangleToggle;
            if (triangleToggle) {
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                type = "Intake";
            } else {
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                type = "Shoot";
            }
        }

        // Circle — toggle intake on/idle
        if (circle && !prevCircle) {
            circleToggle = !circleToggle;
            if (circleToggle) Intake.INSTANCE.on();
            else              Intake.INSTANCE.idle();
        }

        // DpadUp — transfer flick
        if (dpadUp && !prevDpadUp && flickState == FlickState.IDLE) {
            Transfer.INSTANCE.transferUp();
            flickTimer.reset();
            flickState = FlickState.UP;
        }

        // DpadLeft/Right/Down — jump to specific positions
        if (dpadLeft  && !prevDpadLeft)  Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);
        if (dpadRight && !prevDpadRight) Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_TWO);
        if (dpadDown  && !prevDpadDown)  Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_THREE);

        // Square — manually mark current position as empty
        if (square && !prevSquare) {
            Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
        }

        // Drivetrain
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // ── Shoot-three state machine ─────────────────────────────────────
        switch (shootState) {
            case SET_POS:
                Spindexer.INSTANCE.setToPosition(SHOOT_ORDER[shootBallIndex]);
                shootTimer.reset();
                shootState = ShootState.WAIT_POS;
                break;
            case WAIT_POS:
                if (shootTimer.seconds() >= 0.5) {
                    Transfer.INSTANCE.transferUp();
                    shootTimer.reset();
                    shootState = ShootState.TRANSFER_UP;
                }
                break;
            case TRANSFER_UP:
                if (shootTimer.seconds() >= 0.5) {
                    Transfer.INSTANCE.transferDown();
                    shootTimer.reset();
                    shootState = ShootState.TRANSFER_DOWN;
                }
                break;
            case TRANSFER_DOWN:
                if (shootTimer.seconds() >= 0.5) {
                    shootState = ShootState.MARK_EMPTY;
                }
                break;
            case MARK_EMPTY:
                Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                shootBallIndex++;
                if (shootBallIndex >= 3) {
                    shootCycle = false;
                    shootState = ShootState.IDLE;
                } else {
                    shootState = ShootState.SET_POS;
                }
                break;
            default:
                break;
        }

        // ── Transfer flick state machine ──────────────────────────────────
        if (flickState == FlickState.UP && flickTimer.seconds() >= 0.5) {
            Transfer.INSTANCE.transferDown();
            flickTimer.reset();
            flickState = FlickState.DOWN;
        } else if (flickState == FlickState.DOWN && flickTimer.seconds() >= 0.5) {
            Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
            flickState = FlickState.IDLE;
        }

        // ── Periodic ─────────────────────────────────────────────────────
        Spindexer.INSTANCE.periodic();
        // dashboard spinAngle override (matches original onUpdate behavior)
        if (spinAngle != 0) Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());

        // ── Telemetry ────────────────────────────────────────────────────
        telemetry.addData("Shoot Cycle",            shootCycle);
        telemetry.addData("Empty",                  Spindexer.INSTANCE.getEmpty());
        telemetry.addData("Full",                   Spindexer.INSTANCE.getFull());
        telemetry.addData("Filled Position",        Spindexer.INSTANCE.filledPosition());
        telemetry.addData("Nearest Free Position",  Spindexer.INSTANCE.freePosition());
        telemetry.addData("Current Color",          Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One",   Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two",   Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Spindexer Position",     Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode",                   Spindexer.INSTANCE.getPositionType());
        telemetry.update();

        // save last button state
        prevLeftBumper  = leftBumper;
        prevRightBumper = rightBumper;
        prevCross       = cross;
        prevTriangle    = triangle;
        prevCircle      = circle;
        prevDpadUp      = dpadUp;
        prevDpadLeft    = dpadLeft;
        prevDpadRight   = dpadRight;
        prevDpadDown    = dpadDown;
        prevSquare      = square;
    }

    @Override
    public void stop() {
        Intake.INSTANCE.idle();
    }
}