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
@TeleOp(name = "TestTurret")
public class TestTurret extends OpMode {

    public static double turretAngle = 0;
    public boolean goalTrack = false;

    // --- Button edge detection ---
    private boolean prevCross  = false;
    private boolean prevSquare = false;

    // --- Toggle ---
    private boolean goalTrackToggle = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
    }

    @Override
    public void start() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)
        );
    }

    @Override
    public void loop() {
        handleButtons();
        updateTurret();
        updateTelemetry();

        Turret.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Button handling
    // -------------------------------------------------------------------------
    private void handleButtons() {
        boolean cross  = gamepad1.cross;
        boolean square = gamepad1.square;

        // Cross — toggle goal tracking on/off
        if (cross && !prevCross) {
            goalTrackToggle = !goalTrackToggle;
            goalTrack = goalTrackToggle;
        }

        // Square — reset odometry
        if (square && !prevSquare) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)
            );
        }

        prevCross  = cross;
        prevSquare = square;
    }

    // -------------------------------------------------------------------------
    // Turret logic
    // -------------------------------------------------------------------------
    private void updateTurret() {
        if (!goalTrack) {
            Turret.INSTANCE.setToAngle(turretAngle);
        } else {
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
        }
        Turret.INSTANCE.setVelocity(0);
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
    private void updateTelemetry() {
        telemetry.addData("x",               Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y",               Pinpoint.INSTANCE.getPosY());
        telemetry.addData("heading",         Pinpoint.INSTANCE.getHeading());
        telemetry.addData("Current Angle",   Turret.INSTANCE.getTurretAngle());
        telemetry.addData("360 Heading",     (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Turret Tracking", goalTrack);
    }
}