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
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

@Config
@TeleOp(name = "TestShooter")
public class TestShooter extends OpMode {

    public static double hoodPosition = 0;
    public static double velocity     = 0;
    public static boolean ppLL        = true;

    // button edge detection
    private boolean prevY, prevSquare;

    // toggles
    private boolean yToggle, squareToggle;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
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
        boolean square = gamepad1.square;

        // Y — toggle transfer up/down
        if (y && !prevY) {
            yToggle = !yToggle;
            if (yToggle) Transfer.INSTANCE.transferUp();
            else         Transfer.INSTANCE.transferDown();
        }

        // Square — toggle ppLL flag
        if (square && !prevSquare) {
            squareToggle = !squareToggle;
            ppLL = squareToggle;
        }

        prevY      = y;
        prevSquare = square;

        // Drivetrain
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Turret
        Turret.INSTANCE.setHoodPosition(hoodPosition);
        Turret.INSTANCE.setVelocity(-velocity);
        Turret.INSTANCE.periodic();

        // Odometry
        Pinpoint.INSTANCE.periodic();

        // Telemetry
        telemetry.addLine("Turret Tracking Odometry");
        telemetry.addData("x",                            Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y",                            Pinpoint.INSTANCE.getPosY());
        telemetry.addData("heading",                      Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading",                  (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Turret Angle Set",             Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Turret Power Set",             Turret.INSTANCE.getTurretPowerSet());
        telemetry.addData("Hood Position",                hoodPosition);
        telemetry.addData("Turret Positions (LL)",        Turret.INSTANCE.headingToTurretPositionLL());
        telemetry.addData("Turret Nonwrapped Angle",      Turret.INSTANCE.getNonWrappedAngleFromEncoder());
        telemetry.addData("Turret Wrapped Angle",         Turret.INSTANCE.getWrappedAngleFromEncoder());
        telemetry.addData("Shooter One Velocity",         Turret.INSTANCE.getVelocity());
        telemetry.addData("Shooter Two Velocity",         Turret.INSTANCE.getVelocityTwo());
        telemetry.addData("Set Velocity",                 velocity);
        telemetry.update();
    }

    @Override
    public void stop() {}
}