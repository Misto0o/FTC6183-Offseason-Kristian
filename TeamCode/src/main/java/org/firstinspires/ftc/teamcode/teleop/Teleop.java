package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.MatchPattern;

@Config
@TeleOp(name = "Teleop", group = "!")
public class Teleop extends OpMode {

    // ── Alliance ──────────────────────────────────────────────────────────────
    private Aliance currentAliance = Aliance.BLUE;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false; // false=intake, true=shoot
    private boolean turretLock   = false; // locks turret to 270°
    private boolean transferDown = true;  // is the transfer arm down?
    private boolean intakeOn     = false;

    // ── Square flywheel state machine ─────────────────────────────────────────
    // 0=off  1=spinning up  2=ready to shoot
    private int     squareState = 0;
    private boolean hasRumbled  = false;

    // ── Transfer flick state machine ──────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastCircle, lastCross, lastSquare, lastTriangle;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft;

    // ─────────────────────────────────────────────────────────────────────────
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
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap); // safe — try-catch inside

        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);

        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.addData("Alliance", currentAliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !lastDpadUp)   currentAliance = Aliance.BLUE;
        if (gamepad1.dpad_down && !lastDpadDown) currentAliance = Aliance.RED;

        lastDpadUp   = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        MatchPattern.tryDetect();

        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.addData("Alliance selected", currentAliance);
        telemetry.addData("Pattern", MatchPattern.getPattern());
        telemetry.addData("Pattern Locked", MatchPattern.isLocked());
        telemetry.update();
    }

    @Override
    public void start() {
        if (currentAliance == Aliance.BLUE) {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0)
            );
        } else {
            Pinpoint.INSTANCE.updatePosition(
                    new Pose2D(DistanceUnit.INCH, 8.5, 9, AngleUnit.DEGREES, 180)
            );
        }
    }

    @Override
    public void loop() {
        // ── Read inputs ───────────────────────────────────────────────────────
        boolean circle       = gamepad1.circle;
        boolean cross        = gamepad1.cross;
        boolean square       = gamepad1.square;
        boolean triangle     = gamepad1.triangle;
        boolean leftBumper   = gamepad1.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper;
        boolean dpadUp       = gamepad1.dpad_up;
        boolean dpadDown     = gamepad1.dpad_down;
        boolean dpadLeft     = gamepad1.dpad_left;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;

        // ── Circle: toggle intake ─────────────────────────────────────────────
        if (circle && !lastCircle) {
            intakeOn = !intakeOn;
            if (intakeOn) Intake.INSTANCE.on();
            else          Intake.INSTANCE.idle();
        }

        // ── Cross: manual transfer flick (only when transfer is down and idle) ─
        if (cross && !lastCross && flickState == FlickState.IDLE && transferDown) {
            transferDown = false;
            Transfer.INSTANCE.transferUp();
            flickTimer.reset();
            flickState = FlickState.WAIT_UP;
        }

        // ── Triangle: toggle intake / shoot mode ──────────────────────────────
        if (triangle && !lastTriangle) {
            shootMode = !shootMode;
            if (shootMode) {
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            } else {
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                Intake.INSTANCE.on();
                intakeOn = true;
            }
        }

        // ── Square: flywheel 3-state cycle ────────────────────────────────────
        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    // Press 1: start spinning up
                    hasRumbled  = false;
                    squareState = 1;
                    break;
                case 2:
                    // Press 2: shoot — trigger flick if arm is down
                    if (flickState == FlickState.IDLE && transferDown) {
                        transferDown = false;
                        Transfer.INSTANCE.transferUp();
                        flickTimer.reset();
                        flickState  = FlickState.WAIT_UP;
                        squareState = 1; // re-arm for next shot
                        hasRumbled  = false;
                    }
                    break;
                case 1:
                default:
                    // Press during spin-up: emergency kill
                    Turret.INSTANCE.setVelocity(0);
                    squareState = 0;
                    hasRumbled  = false;
                    break;
            }
        }

        // ── DPad Up: turret lock toggle (in match) ────────────────────────────
        if (dpadUp && !lastDpadUp) turretLock = !turretLock;

        // ── DPad Down: zero turret angle offset ───────────────────────────────
        if (dpadDown && !lastDpadDown) Turret.INSTANCE.zeroAngleOffset();

        // ── DPad Left: reverse intake toggle ─────────────────────────────────
        if (dpadLeft && !lastDpadLeft) {
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }

        // ── Triggers: fine-tune turret angle offset ───────────────────────────
        if (rightTrigger) Turret.INSTANCE.updateAngleOffset(-0.1);
        if (leftTrigger)  Turret.INSTANCE.updateAngleOffset(0.1);

        // ── Bumpers: manual spindexer position ────────────────────────────────
        if (leftBumper  && !lastLeftBumper)  Spindexer.Position.next();
        if (rightBumper && !lastRightBumper) Spindexer.Position.previous();

        // ── Transfer flick state machine ──────────────────────────────────────
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
                            Spindexer.DetectedColor.EMPTY
                    );
                    transferDown = true;
                    flickState   = FlickState.IDLE;
                }
                break;
            default:
                break;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        // ── Spindexer auto-rotate ─────────────────────────────────────────────
        if (!shootMode) {
            // Intake: always seek next free slot
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) {
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
            }
        } else {
            // Shoot: seek filled slot only when arm is down (safe to index)
            int filled = Spindexer.INSTANCE.filledPosition();
            if (filled != -1 && transferDown) {
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
            } else {
                Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
            }
        }

        // ── Velocity & hood from interpolator table ───────────────────────────
        double targetVelocity;
        double targetHood;
        if (!shootMode) {
            targetVelocity = 500;  // warm idle during intake
            targetHood     = 1.0;
        } else {
            targetVelocity = Turret.INSTANCE.distanceToVelocity(
                    Pinpoint.INSTANCE.getPosX(),
                    Pinpoint.INSTANCE.getPosY(),
                    currentAliance
            );
            targetHood = Turret.INSTANCE.distanceToPosition(
                    Pinpoint.INSTANCE.getPosX(),
                    Pinpoint.INSTANCE.getPosY(),
                    currentAliance
            );
        }

        // ── Turret aim ────────────────────────────────────────────────────────
        if (turretLock || !shootMode) {
            Turret.INSTANCE.setToAngle(270); // park facing back
        } else {
            Turret.INSTANCE.followGoalOdometryPositional(currentAliance);
        }

        // Apply velocity only if flywheel is active
        if (squareState > 0) {
            Turret.INSTANCE.setVelocity(targetVelocity);
        } else {
            Turret.INSTANCE.setVelocity(0);
        }
        Turret.INSTANCE.setHoodPosition(targetHood);

        // ── Rumble: at speed → rumbleBlips(1) ────────────────────────────────
        if (squareState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - targetVelocity) < Turret.threshold
                && !hasRumbled) {
            gamepad1.rumbleBlips(1);
            hasRumbled  = true;
            squareState = 2;
        }

        // ── Rumble: ball detected → left=green, right=purple ─────────────────
        Spindexer.DetectedColor detected = Spindexer.INSTANCE.readCurrentColor();
        if (detected == Spindexer.DetectedColor.GREEN) {
            gamepad1.rumble(1.0, 0.0, 200);
        } else if (detected == Spindexer.DetectedColor.PURPLE) {
            gamepad1.rumble(0.0, 1.0, 200);
        }

        // ── Periodic ─────────────────────────────────────────────────────────
        Spindexer.INSTANCE.periodic();
        Turret.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Alliance",           currentAliance);
        telemetry.addData("Mode",               shootMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Turret Lock",        turretLock);
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP" : "READY");
        telemetry.addData("Shooter Velocity",   Turret.INSTANCE.getVelocity());
        telemetry.addData("Target Velocity",    targetVelocity);
        telemetry.addData("Turret Angle",       Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Turret Angle Set",   Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Hood Position",      targetHood);
        telemetry.addData("Transfer Down",      transferDown);
        telemetry.addData("x",                  Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y",                  Pinpoint.INSTANCE.getPosY());
        telemetry.addData("Heading",            (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));
        telemetry.addData("Current Color",      detected);
        telemetry.addData("Ball Pos 1",         Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball Pos 2",         Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball Pos 3",         Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Free Position",      Spindexer.INSTANCE.freePosition());
        telemetry.addData("Spindexer Pos",      Spindexer.INSTANCE.getPosition());
        telemetry.update();

        // ── Save last button state ────────────────────────────────────────────
        lastCircle      = circle;
        lastCross       = cross;
        lastSquare      = square;
        lastTriangle    = triangle;
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp      = dpadUp;
        lastDpadDown    = dpadDown;
        lastDpadLeft    = dpadLeft;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        Limelight.INSTANCE.stop();
    }
}