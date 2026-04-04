package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

@Config
@TeleOp(name = "TestShooter", group = "Tuning")
public class TestShooter extends OpMode {

    // Tune these live on the dashboard
    public static double hoodPosition  = 0.10;
    public static double velocity      = 1000;
    public static double turretAngle   = 270;   // degrees, within 180–360

    // Button edge detection
    private boolean prevY, prevX, prevSquare;

    // Toggles
    private boolean shooterOn   = false;
    private boolean transferUp  = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDown();
        telemetry.addLine("Square = shooter on/off  |  Y = transfer up/down  |  X = single flick");
        telemetry.update();
    }

    @Override
    public void start() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)
        );
    }

    @Override
    public void loop() {
        boolean y      = gamepad1.y;
        boolean x      = gamepad1.x;
        boolean square = gamepad1.square;

        // Square — toggle shooter on/off
        if (square && !prevSquare) {
            shooterOn = !shooterOn;
            Turret.INSTANCE.setVelocity(shooterOn ? velocity : 0);
        }

        // Y — toggle transfer up/down
        if (y && !prevY) {
            transferUp = !transferUp;
            if (transferUp) Transfer.INSTANCE.transferUp();
            else            Transfer.INSTANCE.transferDown();
        }

        // X — single flick (up then back down immediately, for manual shot testing)
        if (x && !prevX) {
            Transfer.INSTANCE.transferUp();
        }
        if (!x && prevX) {
            Transfer.INSTANCE.transferDown();
        }

        prevY      = y;
        prevX      = x;
        prevSquare = square;

        // Keep velocity in sync if dashboard value changed while shooter is on
        if (shooterOn) Turret.INSTANCE.setVelocity(velocity);

        // Drivetrain
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // Turret — set angle and hood from dashboard, then run periodic
        Turret.INSTANCE.setToAngle(turretAngle);
        Turret.INSTANCE.setHoodPosition(hoodPosition);
        Turret.INSTANCE.periodic();

        Pinpoint.INSTANCE.periodic();

        // Telemetry
        double actual   = Turret.INSTANCE.getVelocity();
        double target   = shooterOn ? velocity : 0;
        double error    = target - actual;
        boolean ready   = shooterOn && Math.abs(error) < Turret.threshold;

        telemetry.addLine("── SHOOTER ──────────────────────────────");
        telemetry.addData("Shooter",          shooterOn ? "ON" : "OFF");
        telemetry.addData("Velocity (target)", (int) target);
        telemetry.addData("Velocity (actual)", (int) actual);
        telemetry.addData("Error",             (int) error);
        telemetry.addData("READY",             ready ? "YES - flick!" : "no");

        telemetry.addLine("── TURRET ───────────────────────────────");
        telemetry.addData("Angle (set)",       turretAngle);
        telemetry.addData("Angle (actual)",    Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Hood",              hoodPosition);

        telemetry.addLine("── ODOMETRY ─────────────────────────────");
        telemetry.addData("X",                 Pinpoint.INSTANCE.getPosX());
        telemetry.addData("Y",                 Pinpoint.INSTANCE.getPosY());
        telemetry.addData("Heading",           Pinpoint.INSTANCE.getHeading());

        telemetry.update();
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Turret.INSTANCE.periodic();
    }
}