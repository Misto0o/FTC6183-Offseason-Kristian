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
@TeleOp(name = "TestTurret_Drive", group = "Tuning")
public class TestTurret extends OpMode {

    // ── CALIBRATION TUNING VALUES ──────────────────────────────────────────
    public static double goalX = 0;                // Blue goal X (TUNE THIS)
    public static double goalY = 144;               // Blue goal Y (TUNE THIS)
    public static double robotStartX = 135.5;       // Starting position X
    public static double robotStartY = 9;           // Starting position Y
    public static double robotStartHeading = 0;     // Starting heading in degrees

    public boolean showDebug = false;               // Show detailed calculations

    // --- Button edge detection ---
    private boolean prevSquare = false;
    private boolean prevCircle = false;
    private boolean prevDpadUp = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);

        telemetry.addLine("TestTurret with Drivetrain");
        telemetry.addLine("Left Stick = Drive");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("SQUARE = Toggle Goal Tracking");
        telemetry.addLine("CIRCLE = Toggle Debug Output");
        telemetry.addLine("DPAD UP = Reset Position to Start");
        telemetry.update();
    }

    @Override
    public void start() {
        resetPosition();
    }

    private void resetPosition() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, robotStartX, robotStartY, AngleUnit.DEGREES, robotStartHeading)
        );
    }

    private boolean goalTrackEnabled = false;

    @Override
    public void loop() {
        // ── Read inputs ───────────────────────────────────────────────────────
        boolean square = gamepad1.square;
        boolean circle = gamepad1.circle;
        boolean dpadUp = gamepad1.dpad_up;

        // ── Button handling ────────────────────────────────────────────────────
        if (square && !prevSquare) {
            goalTrackEnabled = !goalTrackEnabled;
            gamepad1.rumbleBlips(goalTrackEnabled ? 2 : 1);
        }

        if (circle && !prevCircle) {
            showDebug = !showDebug;
        }

        if (dpadUp && !prevDpadUp) {
            resetPosition();
        }

        prevSquare = square;
        prevCircle = circle;
        prevDpadUp = dpadUp;

        // ── Drivetrain control ────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );


        if (gamepad1.left_trigger > 0.5) {
            Turret.INSTANCE.calibrateToParkedPosition();
            telemetry.addLine("TURRET CALIBRATED");
        }


        // ── Turret control ────────────────────────────────────────────────────
        // Sync tunable values to Turret class
        Turret.BLUE_GOAL_X = goalX;
        Turret.BLUE_GOAL_Y = goalY;
        Turret.DEBUG_AIM = showDebug;

        // Always aim at goal if enabled
        if (goalTrackEnabled) {
            Turret.INSTANCE.aimAtGoal(Aliance.BLUE, 20);
        } else {
            // Otherwise point turret forward (0°)
            Turret.INSTANCE.setToAngle(0);
        }

        Turret.INSTANCE.setVelocity(0);  // Don't spin flywheels during tuning
        Turret.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();

        // ── Telemetry ──────────────────────────────────────────────────────────
        telemetry.addLine("========== POSITION ==========");
        telemetry.addData("Robot X", String.format("%.1f", Pinpoint.INSTANCE.getPosX()));
        telemetry.addData("Robot Y", String.format("%.1f", Pinpoint.INSTANCE.getPosY()));
        telemetry.addData("Robot Heading", String.format("%.1f°", Pinpoint.INSTANCE.getHeading()));
        telemetry.addLine("");

        telemetry.addLine("========== TURRET ==========");
        telemetry.addData("Turret Angle", String.format("%.1f°", Turret.INSTANCE.getTurretAngle()));
        telemetry.addData("Goal Track", goalTrackEnabled ? "ON" : "OFF");
        telemetry.addLine("");

        telemetry.addLine("========== GOAL COORDINATES ==========");
        telemetry.addData("Goal X", String.format("%.1f", goalX));
        telemetry.addData("Goal Y", String.format("%.1f", goalY));
        telemetry.addData("Delta X", String.format("%.1f", goalX - Pinpoint.INSTANCE.getPosX()));
        telemetry.addData("Delta Y", String.format("%.1f", goalY - Pinpoint.INSTANCE.getPosY()));
        telemetry.addLine("");

        telemetry.addLine("========== INSTRUCTIONS ==========");
        telemetry.addLine("1. Drive to the GOAL location");
        telemetry.addLine("2. Enable SQUARE to toggle goal tracking");
        telemetry.addLine("3. Turret should point AT the goal");
        telemetry.addLine("4. Note position when turret points right");
        telemetry.addLine("5. Update goalX and goalY with that position");
        telemetry.addLine("");
        telemetry.addLine("DPAD UP = Reset to start position");
        telemetry.addLine("CIRCLE = Toggle debug output");

        telemetry.update();
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
    }
}