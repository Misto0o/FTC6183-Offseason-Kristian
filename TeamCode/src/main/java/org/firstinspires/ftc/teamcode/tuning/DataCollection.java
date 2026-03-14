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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

@Config
@TeleOp(name = "DataCollection")
public class DataCollection extends OpMode {

    public static double velocity        = 0;
    public static double hoodPosition    = 0;
    public static double offsetVelocity  = 0;
    public static double shootThreshold  = 100;

    public boolean shootcycle = false;

    private boolean prevCircle, prevCross, prevTriangle, prevSquare;
    private boolean prevDpadLeft, prevLeftBumper, prevRightBumper;

    private boolean intakeToggle, modeToggle, reverseToggle;

    private double transferFlickTimer = -1;
    private int    transferFlickStage =  0;

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
        tickTransferFlick();
        updateSpindexer();
        updateTurret();
        updateTelemetry();

        Pinpoint.INSTANCE.periodic();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

        telemetry.update();
    }

    private void handleButtons() {
        boolean circle      = gamepad1.circle;
        boolean cross       = gamepad1.cross;
        boolean triangle    = gamepad1.triangle;
        boolean square      = gamepad1.square;
        boolean dpadLeft    = gamepad1.dpad_left;
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        // Circle — toggle intake on/off
        if (circle && !prevCircle) {
            intakeToggle = !intakeToggle;
            if (intakeToggle) Intake.INSTANCE.on();
            else              Intake.INSTANCE.idle();
        }

        // Cross — trigger transfer flick
        if (cross && !prevCross) scheduleTransferFlick();

        // Triangle — toggle intake/shoot mode
        if (triangle && !prevTriangle) {
            modeToggle = !modeToggle;
            Spindexer.INSTANCE.setPositionType(
                    modeToggle ? Spindexer.PositionType.SHOOT : Spindexer.PositionType.INTAKE
            );
        }

        // Square — reset odometry
        if (square && !prevSquare) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 8.5, 8.875, AngleUnit.DEGREES, 90)
            );
        }

        // DpadLeft — toggle reverse intake
        if (dpadLeft && !prevDpadLeft) {
            reverseToggle = !reverseToggle;
            if (reverseToggle) Intake.INSTANCE.reverse();
            else               Intake.INSTANCE.idle();
        }

        if (leftBumper  && !prevLeftBumper)  Spindexer.Position.next();
        if (rightBumper && !prevRightBumper) Spindexer.Position.previous();

        prevCircle      = circle;
        prevCross       = cross;
        prevTriangle    = triangle;
        prevSquare      = square;
        prevDpadLeft    = dpadLeft;
        prevLeftBumper  = leftBumper;
        prevRightBumper = rightBumper;
    }

    private void scheduleTransferFlick() {
        if (transferFlickStage != 0) return;
        transferFlickTimer = getRuntime();
        transferFlickStage = 1;
        Transfer.INSTANCE.transferUp();
    }

    private void tickTransferFlick() {
        if (transferFlickStage == 0) return;
        double elapsed = getRuntime() - transferFlickTimer;
        if (transferFlickStage == 1 && elapsed >= 0.5) {
            Transfer.INSTANCE.transferDown();
            transferFlickTimer = getRuntime();
            transferFlickStage = 2;
        } else if (transferFlickStage == 2 && elapsed >= 0.5) {
            Spindexer.INSTANCE.setColor(
                    Spindexer.INSTANCE.getPosition(),
                    Spindexer.DetectedColor.EMPTY
            );
            transferFlickStage = 0;
        }
    }

    private void updateSpindexer() {
        if (Spindexer.INSTANCE.freePosition() != -1
                && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && !shootcycle) {
            Spindexer.INSTANCE.setToPosition(
                    Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]
            );
        } else if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT
                && Spindexer.INSTANCE.filledPosition() != -1
                && !shootcycle) {
            Spindexer.INSTANCE.setToPosition(
                    Spindexer.Position.values()[Spindexer.INSTANCE.filledPosition()]
            );
        } else {
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
        }
    }

    private void updateTurret() {
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            velocity = 500;
            Turret.INSTANCE.setToAngle(270);
        } else {
            velocity = Turret.INSTANCE.distanceToVelocity(
                    Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY(), Aliance.BLUE
            );
        }
        hoodPosition = Turret.INSTANCE.distanceToPosition(
                Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY(), Aliance.BLUE
        );
        Turret.INSTANCE.setVelocity(velocity);
        Turret.INSTANCE.setHoodPosition(hoodPosition);
    }

    private void updateTelemetry() {
        telemetry.addData("Turret Angle Set",       Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("x",                      Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y",                      Pinpoint.INSTANCE.getPosY());
        telemetry.addData("Heading",                Pinpoint.INSTANCE.getHeading());
        telemetry.addData("360 Heading",            (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Turret Power Set",       Turret.INSTANCE.getTurretPowerSet());
        telemetry.addData("Shooter Velocity",       Turret.INSTANCE.getVelocity());
        telemetry.addData("Set Velocity",           velocity);
        telemetry.addData("Hood Position",          hoodPosition);
        telemetry.addData("Current Color",          Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One",   Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two",   Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Spindexer Position",     Spindexer.INSTANCE.getPosition());
        telemetry.addData("Mode",                   Spindexer.INSTANCE.getPositionType());
        telemetry.addData("Empty",                  Spindexer.INSTANCE.getEmpty());
        telemetry.addData("Full",                   Spindexer.INSTANCE.getFull());
    }
}