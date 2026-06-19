package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

@Config
@TeleOp(name = "TestTurret", group = "Tuning")
public class TestTurret extends OpMode {

    public static double goalX = 0; // Blue
    public static double goalY = 144;
    public static double robotStartX = 135.5; // Blue starting corner
    public static double robotStartY = 9;

    private boolean goalTrackEnabled = false;
    private double recordedRightLimit = -1;
    private double recordedLeftLimit  = -1;

    private boolean prevSquare   = false;
    private boolean prevCircle   = false;
    private boolean prevTriangle = false;
    private boolean prevDpadUp   = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight= false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);

        telemetry.addLine("Initializing — point turret at intake, then hold ✕ to calibrate.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Same calibration as teleop — hold cross to snap offset to parked position
        if (gamepad1.cross) {
            Turret.INSTANCE.calibrateToParkedPosition();
            gamepad1.rumbleBlips(2);
        }

        telemetry.addLine("Hold ✕ with turret at intake position to calibrate.");
        telemetry.addData("Turret Angle", String.format("%.2f°", Turret.INSTANCE.getTurretAngle()));
        telemetry.addData("TurretOffset", Turret.INSTANCE.getTurretOffSet());
        telemetry.update();
    }

    @Override
    public void start() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, robotStartX, robotStartY, AngleUnit.DEGREES, 0));
        // Park turret at 270 on start
        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
    }

    @Override
    public void loop() {
        Pinpoint.INSTANCE.periodic();

        boolean square    = gamepad1.square;
        boolean circle    = gamepad1.circle;
        boolean triangle  = gamepad1.triangle;
        boolean dpadUp    = gamepad1.dpad_up;
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // ── SQUARE: toggle goal tracking ──────────────────────────────────────
        if (square && !prevSquare) {
            goalTrackEnabled = !goalTrackEnabled;
            gamepad1.rumbleBlips(goalTrackEnabled ? 2 : 1);
        }

        // ── CIRCLE: return turret to parked (270°) ────────────────────────────
        if (circle && !prevCircle) {
            goalTrackEnabled = false;
            Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
            gamepad1.rumbleBlips(1);
        }

        // ── TRIANGLE: re-calibrate parked position ────────────────────────────
        if (triangle && !prevTriangle) {
            Turret.INSTANCE.calibrateToParkedPosition();
            gamepad1.rumbleBlips(2);
        }

        // ── DPAD RIGHT: record right physical limit ───────────────────────────
        if (dpadRight && !prevDpadRight) {
            recordedRightLimit = Turret.INSTANCE.getTurretAngle();
            gamepad1.rumbleBlips(1);
        }

        // ── DPAD LEFT: record left physical limit ─────────────────────────────
        if (dpadLeft && !prevDpadLeft) {
            recordedLeftLimit = Turret.INSTANCE.getTurretAngle();
            gamepad1.rumbleBlips(1);
        }

        // ── DPAD UP: reset odometry ───────────────────────────────────────────
        if (dpadUp && !prevDpadUp) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, robotStartX, robotStartY, AngleUnit.DEGREES, 0));
        }

        prevSquare    = square;
        prevCircle    = circle;
        prevTriangle  = triangle;
        prevDpadUp    = dpadUp;
        prevDpadLeft  = dpadLeft;
        prevDpadRight = dpadRight;

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // ── Turret control ────────────────────────────────────────────────────
        Turret.BLUE_GOAL_X = goalX;
        Turret.BLUE_GOAL_Y = goalY;

        if (goalTrackEnabled) {
            Turret.INSTANCE.aimAtGoal(Aliance.BLUE, 20);
        }
        // When not tracking, turret holds whatever setToAngle was last called with.
        // Motor is still powered by periodic() so it holds position.
        // Physically push it to find limits — PD will resist but you can overpower it.

        Turret.INSTANCE.setVelocity(0);
        Turret.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("════ TURRET ════");
        telemetry.addData("Angle (actual)", String.format("%.2f°", Turret.INSTANCE.getTurretAngle()));
        telemetry.addData("Target",         String.format("%.2f°", Turret.INSTANCE.getTurretAngleSet()));
        telemetry.addData("Offset",         String.format("%.4f",  Turret.INSTANCE.getTurretOffSet()));
        telemetry.addData("Goal Track",     goalTrackEnabled ? "ON ✓" : "OFF");
        telemetry.addLine("");

        telemetry.addLine("════ LIMITS (push turret to end, press DPad) ════");
        telemetry.addData("Right limit (DPad →)", recordedRightLimit >= 0
                ? String.format("%.2f°", recordedRightLimit) : "not recorded");
        telemetry.addData("Left limit  (DPad ←)", recordedLeftLimit >= 0
                ? String.format("%.2f°", recordedLeftLimit)  : "not recorded");
        telemetry.addLine("");

        telemetry.addLine("════ POSITION ════");
        telemetry.addData("X",       String.format("%.1f", Pinpoint.INSTANCE.getPosX()));
        telemetry.addData("Y",       String.format("%.1f", Pinpoint.INSTANCE.getPosY()));
        telemetry.addData("Heading", String.format("%.1f°", Pinpoint.INSTANCE.getHeading()));
        telemetry.addLine("");

        telemetry.addLine("════ CONTROLS ════");
        telemetry.addLine("□  = Toggle goal tracking");
        telemetry.addLine("○  = Return to parked (270°)");
        telemetry.addLine("△  = Recalibrate parked position");
        telemetry.addLine("→  = Record RIGHT physical limit");
        telemetry.addLine("←  = Record LEFT physical limit");
        telemetry.addLine("↑  = Reset odometry");

        telemetry.update();
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
    }
}