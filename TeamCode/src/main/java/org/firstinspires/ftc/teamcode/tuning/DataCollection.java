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
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Intake;

import java.util.ArrayList;
import java.util.Locale;

@Config
@TeleOp(name = "DataCollection", group = "Tuning")
public class DataCollection extends OpMode {

    public static double shootVelocity = 1000;
    public static double hoodPosition  = 0.1;

    private double lockedX = 0;
    private double lockedY = 0;
    private boolean positionLocked = false;

    private final ArrayList<String> shooterLog = new ArrayList<>();
    private final ArrayList<String> hoodLog    = new ArrayList<>();

    private boolean lastSquare, lastCross, lastTriangle;
    private int squareState = 0;
    private boolean hasRumbled = false;
    private final ElapsedTime rumbleTimer = new ElapsedTime();
    private boolean lastCircle;
    private boolean intakeOn = false;
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;
    public static double intakeDwellSec = 0.3;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Transfer.INSTANCE.transferDown();
        telemetry.addLine("Ready — drive to shooting spot then press Square to lock position");
        telemetry.update();
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
    }

    @Override
    public void start() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        boolean square   = gamepad1.square;
        boolean cross    = gamepad1.cross;
        boolean triangle = gamepad1.triangle;

        Pinpoint.INSTANCE.periodic();

        double px = Pinpoint.INSTANCE.getPosX();
        double py = Pinpoint.INSTANCE.getPosY();

        // ── Square: lock position ─────────────────────────────────────────────
        if (square && !lastSquare) {
            lockedX        = px;
            lockedY        = py;
            positionLocked = true;
            squareState    = 0;
            hasRumbled     = false;
            Turret.INSTANCE.setVelocity(0);
        }

        boolean circle = gamepad1.circle;

        if (circle && !lastCircle) {
            intakeOn = !intakeOn;
            if (intakeOn) Intake.INSTANCE.on();
            else          Intake.INSTANCE.idle();
        }

        // ── Triangle: spin up / cancel ────────────────────────────────────────
        if (triangle && !lastTriangle && positionLocked) {
            if (squareState == 0) {
                squareState = 1;
                hasRumbled  = false;
                Turret.INSTANCE.setVelocity(shootVelocity);
                Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            } else {
                squareState = 0;
                Turret.INSTANCE.setVelocity(0);
            }
        }

        // ── Cross: log confirmed data point ──────────────────────────────────
        if (cross && !lastCross && positionLocked && squareState == 2) {
            String shooterLine = String.format(Locale.US,
                    "shooterBlue.addPoint(%.1f, %.1f, %.0f);", lockedX, lockedY, shootVelocity);
            String hoodLine = String.format(Locale.US,
                    "hoodBlue.addPoint(%.1f, %.1f, %.2f);", lockedX, lockedY, hoodPosition);
            shooterLog.add(shooterLine);
            hoodLog.add(hoodLine);
            gamepad1.rumbleBlips(2);
        }

        // ── Turret tracking ───────────────────────────────────────────────────
        if (squareState > 0 && positionLocked) {
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            Turret.INSTANCE.setVelocity(shootVelocity);
            Turret.INSTANCE.setHoodPosition(hoodPosition);
        } else {
            Turret.INSTANCE.setToAngle(270);
            Turret.INSTANCE.setHoodPosition(1.0);
        }

        // ── Rumble when ready ─────────────────────────────────────────────────
        if (squareState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - shootVelocity) < Turret.threshold
                && !hasRumbled) {
            gamepad1.rumbleBlips(1);
            hasRumbled  = true;
            squareState = 2;
        }

        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE && intakeOn) {
            Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
            if (!dwelling) {
                dwelling = true;
                dwellTimer.reset();
            } else if (dwellTimer.seconds() >= intakeDwellSec) {
                Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                dwelling = false;
                Spindexer.INSTANCE.periodic();
                if (!Spindexer.INSTANCE.getFull()) {
                    int free = Spindexer.INSTANCE.freePosition();
                    if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                } else {
                    intakeOn = false;
                    Intake.INSTANCE.idle();
                }
            }
        }

        Turret.INSTANCE.periodic();
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x);

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("── CONTROLS ─────────────────────────────");
        telemetry.addLine("Square = lock position | Triangle = spin up/cancel");
        telemetry.addLine("Cross = log point (only when ready)");
        telemetry.addLine("Tune shootVelocity + hoodPosition on dashboard");
        telemetry.addLine("Circle = toggle intake");

        telemetry.addLine("── POSITION ─────────────────────────────");
        telemetry.addData("Current X", String.format(Locale.US, "%.1f", px));
        telemetry.addData("Current Y", String.format(Locale.US, "%.1f", py));
        telemetry.addData("Locked X",  positionLocked ? String.format(Locale.US, "%.1f", lockedX) : "NOT LOCKED");
        telemetry.addData("Locked Y",  positionLocked ? String.format(Locale.US, "%.1f", lockedY) : "NOT LOCKED");

        telemetry.addLine("── SHOOTER ──────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP..." : "READY — press Cross to log");
        telemetry.addData("Velocity (actual)", (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)", (int) shootVelocity);
        telemetry.addData("Hood",              hoodPosition);
        telemetry.addData("Turret Angle",      Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Balls", Spindexer.INSTANCE.getBallAtPosition()[0] + " " +
                Spindexer.INSTANCE.getBallAtPosition()[1] + " " +
                Spindexer.INSTANCE.getBallAtPosition()[2]);

        telemetry.addLine("── LOGGED POINTS ────────────────────────");
        telemetry.addData("Points logged", shooterLog.size());
        for (int i = 0; i < shooterLog.size(); i++) {
            telemetry.addData("S" + i, shooterLog.get(i));
            telemetry.addData("H" + i, hoodLog.get(i));
        }

        telemetry.update();

        lastSquare   = square;
        lastCross    = cross;
        lastTriangle = triangle;
        lastCircle = circle;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
    }
}