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

@Config
@TeleOp(name = "FullTest")
public class FullTest extends OpMode {

    public static double hoodPosition = 1;
    public static double velocity     = 0;
    public static String type         = "";

    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;

    private boolean circleToggle;
    private boolean crossToggle;
    private boolean squareToggle;
    private boolean triangleToggle;

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
        Limelight.INSTANCE.initialize(hardwareMap);

        type = "Intake";
        Intake.INSTANCE.on();
    }

    @Override
    public void loop() {
        boolean circle      = gamepad1.circle;
        boolean cross       = gamepad1.cross;
        boolean square      = gamepad1.square;
        boolean triangle    = gamepad1.triangle;
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        // ── Circle: toggle shoot / intake mode ────────────────────────────
        if (circle && !lastCircle) {
            circleToggle = !circleToggle;
            if (circleToggle) {
                type = "Shoot";
                Intake.INSTANCE.idle();
                Turret.INSTANCE.setVelocity(1000);
                flickState = FlickState.UP;
                Transfer.INSTANCE.transferUp();
                flickTimer.reset();
            } else {
                type = "Intake";
                Intake.INSTANCE.on();
                Turret.INSTANCE.setHoodPosition(1);
                Turret.INSTANCE.setVelocity(0);
            }
        }

        // ── Transfer flick state machine ──────────────────────────────────
        if (flickState == FlickState.UP && flickTimer.seconds() >= 0.2) {
            Transfer.INSTANCE.transferDown();
            flickTimer.reset();
            flickState = FlickState.DOWN;
        } else if (flickState == FlickState.DOWN && flickTimer.seconds() >= 0.2) {
            flickState = FlickState.IDLE;
        }

        // ── Cross: transfer up/down toggle ────────────────────────────────
        if (cross && !lastCross) {
            crossToggle = !crossToggle;
            if (crossToggle) Transfer.INSTANCE.transferUp();
            else             Transfer.INSTANCE.transferDown();
        }

        // ── Square: spin sequence ─────────────────────────────────────────
        if (square && !lastSquare) {
            squareToggle = !squareToggle;
            if (squareToggle) {
                Transfer.INSTANCE.transferDown();
                Spindexer.INSTANCE.nextPosition();
            } else {
                Transfer.INSTANCE.transferUp();
            }
        }

        // ── Triangle: toggle type string ──────────────────────────────────
        if (triangle && !lastTriangle) {
            triangleToggle = !triangleToggle;
            type = triangleToggle ? "Shoot" : "Intake";
        }

        // ── Bumpers: spindexer position ───────────────────────────────────
        if (leftBumper  && !lastLeftBumper)  Spindexer.Position.next();
        if (rightBumper && !lastRightBumper) Spindexer.Position.previous();

        // ── Drivetrain ────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // ── Periodic ─────────────────────────────────────────────────────
        Spindexer.INSTANCE.periodic();
        Turret.INSTANCE.setHoodPosition(hoodPosition);
        Turret.INSTANCE.periodic();

        // ── Telemetry ────────────────────────────────────────────────────
        telemetry.addData("Spin Servo Power",       Spindexer.INSTANCE.getPower());
        telemetry.addData("Shooter Velocity",       Turret.INSTANCE.getVelocityTwo());
        telemetry.addData("Set Velocity",           velocity);
        telemetry.addData("Mode",                   type);
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Angle From Blue Tag",    Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP" :
                        Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP" :
                                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG" : "Not found");
        telemetry.addData("Hood Position",          hoodPosition);
        telemetry.addData("Turret Positions",       Turret.INSTANCE.headingToTurretPositionLL());
        telemetry.addData("Turret Relative Angle",  Turret.INSTANCE.getWrappedAngleFromEncoder());
        telemetry.addData("Turret Absolute Angle",  Turret.INSTANCE.getNonWrappedAngleFromEncoder());
        telemetry.addData("Spindexer Position",     Spindexer.INSTANCE.getPosition());
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