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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import java.util.ArrayList;
import java.util.Locale;

@Config
@TeleOp(name = "DataCollection", group = "Tuning")
public class DataCollection extends OpMode {

    /*
 Hello Provolone, here is what you're working on today:

 GOAL: Collect shooter tuning data points for the lookup table.
 Work on Alliance Blue for Today.

 HOW TO USE:
 1. Drive to a spot on the field
 2. Press Triangle to turn on intake — drive under balls until spindexer is full
    (intake turns off automatically when full)
    If the Robot goes into shooter mode when its clearly not full just press Triangle again the robot will rescan and go back to the empty position automatically
 3. Press Square to spin up the flywheel
 4. Wait for the controller to rumble — that means flywheel is at speed
 5. On FTC Dashboard (your laptop), tune ONLY two values:
       shootVelocity — how fast the flywheel spins (start around 1000-1200)
       hoodPosition  — angle of the hood servo (0.0 to 1.0, start around 0.10-0.15)
 6. Press Square again to fire one ball — watch where it goes and adjust
 7. Once all 3 balls are going in consistently, press Cross (X) to log the point
 8. Look at the Driver Station screen under "LOGGED POINTS"
    You will see two lines like this: (Example)
       S0: i.addPoint(135.5, 40.0, 1150);
       H0: i.addPoint(135.5, 40.0, 0.12);
 9. Write these into a Google Doc / write it down / take a photo just DON'T LOOSE THEM
 10. Drive to a new spot and repeat from step 1

 11. If you get a jam just turn on intake again and the bot should go back to shooter mode if you get 3/3 jams
 if not press Dpad left to go back into shooter mode and try again.

 CONTROLS REMINDER:
   Triangle         = intake on/off
   Square           = spin up flywheel / fire one ball / cancel
   Cross            = log current point (only works when flywheel is ready)
   Circle           = kill everything (emergency stop)
   Right Trigger    = toggle turret lock (stops auto-tracking)
   Left Trigger     = reverse intake (unjam)
   Left/Right Bumper = manually rotate spindexer
   Dpad - Left = Manual Shooter Mode

 WHEN DONE:
 Go to the Excel spreadsheet I created and just put in the points
 The S lines go into loadBlueShooter(), the H lines go into loadBlueHood()
*/

    // ── Dashboard tunable — only shooter + hood ───────────────────────────────
    public static double shootVelocity = 1000;
    public static double hoodPosition  = 0.10;

    // ── State ─────────────────────────────────────────────────────────────────
    private int     squareState = 0;
    private boolean hasRumbled  = false;
    private boolean intakeOn    = false;
    private boolean turretLock  = false;
    private boolean transferDown = true;

    // ── Logged points ─────────────────────────────────────────────────────────
    private final ArrayList<String> shooterLog = new ArrayList<>();
    private final ArrayList<String> hoodLog    = new ArrayList<>();

    // ── Transfer flick ────────────────────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Intake dwell ──────────────────────────────────────────────────────────
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;
    public static double intakeDwellSec = 0.3;

    private enum RescanState { IDLE, MOVING, READING }
    private RescanState rescanState = RescanState.IDLE;
    private int rescanIndex = 0;
    private final ElapsedTime rescanTimer = new ElapsedTime();

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastSquare, lastCross, lastTriangle, lastCircle;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastLeftTrigger, lastRightTrigger;
    private boolean lastRightTriggerBtn;

    private boolean lastDpadLeft;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDown();
        Pinpoint.INSTANCE.init(hardwareMap);
        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        telemetry.addLine("Triangle=Intake | Square=Rev/Shoot | Cross=Log point | Circle=Kill");
        telemetry.addLine("Right Trigger=Turret lock | Left Trigger=Reverse intake");
        telemetry.addLine("Bumpers=Spindexer manual");
        telemetry.addLine("Dpad Left = Manual Shoot mode");
        telemetry.update();
    }

    @Override
    public void start() {
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 144 - 8.5, 9, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        boolean square       = gamepad1.square;
        boolean cross        = gamepad1.cross;
        boolean triangle     = gamepad1.triangle;
        boolean circle       = gamepad1.circle;
        boolean dpad_left    = gamepad1.dpad_left;
        boolean leftBumper   = gamepad1.left_bumper;
        boolean rightBumper  = gamepad1.right_bumper;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;

        Pinpoint.INSTANCE.periodic();
        double px = Pinpoint.INSTANCE.getPosX();
        double py = Pinpoint.INSTANCE.getPosY();

        // ── Triangle: intake on / rescan toggle ──────────────────────────────
        if (triangle && !lastTriangle) {
            if (!intakeOn) {
                intakeOn    = true;
                squareState = 0;
                hasRumbled  = false;
                dwelling    = false;
                Turret.INSTANCE.setVelocity(0);
                Turret.INSTANCE.setToAngle(270);
                Intake.INSTANCE.on();
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                int free = Spindexer.INSTANCE.freePosition();
                if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            } else {
                // Second press — non-blocking rescan
                Intake.INSTANCE.idle();
                intakeOn    = false;
                dwelling    = false;
                rescanIndex = 0;
                rescanState = RescanState.MOVING;
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[0]);
                rescanTimer.reset();
            }
        }

        switch (rescanState) {
            case MOVING:
                if (rescanTimer.seconds() >= 0.15) {
                    Spindexer.INSTANCE.setColor(
                            Spindexer.Position.values()[rescanIndex],
                            Spindexer.INSTANCE.readCurrentColor());
                    rescanState = RescanState.READING;
                    rescanTimer.reset();
                }
                break;
            case READING:
                rescanIndex++;
                if (rescanIndex >= Spindexer.Position.values().length) {
                    Spindexer.INSTANCE.periodic();
                    int free = Spindexer.INSTANCE.freePosition();
                    if (free != -1) {
                        intakeOn = true;
                        Intake.INSTANCE.on();
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                    }
                    rescanState = RescanState.IDLE;
                } else {
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[rescanIndex]);
                    rescanTimer.reset();
                    rescanState = RescanState.MOVING;
                }
                break;
            default:
                break;
        }

        // ── Circle: kill everything ───────────────────────────────────────────
        if (circle && !lastCircle) {
            intakeOn     = false;
            squareState  = 0;
            hasRumbled   = false;
            flickState   = FlickState.IDLE;
            transferDown = true;
            dwelling     = false;
            Intake.INSTANCE.idle();
            Turret.INSTANCE.setVelocity(0);
            Transfer.INSTANCE.transferDown();
            Turret.INSTANCE.setToAngle(270);
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            intakeOn     = false;
            squareState  = 1;
            hasRumbled   = false;
            dwelling     = false;
            Intake.INSTANCE.idle();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            int next = filledPosition();
            if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
        }

        // ── Square: flywheel cycle (same as teleop) ───────────────────────────
        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    squareState = 1;
                    hasRumbled  = false;
                    intakeOn    = false;
                    Intake.INSTANCE.idle();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    int next = filledPosition();
                    if (next != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[next]);
                    break;
                case 2:
                    // Fire one ball
                    if (flickState == FlickState.IDLE && transferDown) {
                        transferDown = false;
                        Transfer.INSTANCE.transferUp();
                        flickTimer.reset();
                        flickState  = FlickState.WAIT_UP;
                        squareState = 1;
                        hasRumbled  = false;
                    }
                    break;
                case 1:
                default:
                    Turret.INSTANCE.setVelocity(0);
                    squareState = 0;
                    hasRumbled  = false;
                    break;
            }
        }

        // ── Cross: log current position + velocity + hood ─────────────────────
        if (cross && !lastCross && squareState == 2) {
            String shooterLine = String.format(Locale.US,
                    "i.addPoint(%.1f, %.1f, %.0f);", px, py, shootVelocity);
            String hoodLine = String.format(Locale.US,
                    "i.addPoint(%.1f, %.1f, %.2f);", px, py, hoodPosition);
            shooterLog.add(shooterLine);
            hoodLog.add(hoodLine);
            gamepad1.rumbleBlips(2);
        }

        // ── Right trigger: turret lock toggle ────────────────────────────────
        if (rightTrigger && !lastRightTriggerBtn) turretLock = !turretLock;

        // ── Left trigger: reverse intake ─────────────────────────────────────
        if (leftTrigger && !lastLeftTrigger) {
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }

        // ── Bumpers: manual spindexer ─────────────────────────────────────────
        if (leftBumper  && !lastLeftBumper) {
            Spindexer.Position.next();
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
        }
        if (rightBumper && !lastRightBumper) {
            Spindexer.Position.previous();
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
        }

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
                    int shotSlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                    Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[shotSlot]);
                    flickTimer.reset();
                    flickState   = FlickState.IDLE;
                    transferDown = true;
                    Spindexer.INSTANCE.setColor(
                            Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    Spindexer.INSTANCE.periodic();
                    int nextFilled = filledPosition();
                    if (nextFilled != -1)
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[nextFilled]);
                }
                break;
            default:
                break;
        }

        // ── Intake dwell logic ────────────────────────────────────────────────
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE
                && flickState == FlickState.IDLE && intakeOn) {
            Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
            if (!dwelling) {
                dwelling = true;
                dwellTimer.reset();
            } else if (dwellTimer.seconds() >= intakeDwellSec) {
                Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                dwelling = false;
                Spindexer.INSTANCE.periodic();
                if (Spindexer.INSTANCE.getFull()) {
                    intakeOn = false;
                    Intake.INSTANCE.idle();
                } else {
                    int free = Spindexer.INSTANCE.freePosition();
                    if (free != -1) Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                }
            }
        }

        // ── Turret tracking + velocity + hood ────────────────────────────────
        if (squareState > 0) {
            if (!turretLock) {
                Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            } else {
                Turret.INSTANCE.setToAngle(270);
            }
            Turret.INSTANCE.setVelocity(shootVelocity);
            Turret.INSTANCE.setHoodPosition(hoodPosition);
        } else {
            Turret.INSTANCE.setToAngle(270);
            Turret.INSTANCE.setVelocity(0);
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

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x * Drivetrain.getInstance().getTurnSpeed()
        );

        Turret.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("── CONTROLS ─────────────────────────────");
        telemetry.addLine("Triangle=Intake | Square=Rev/Shoot | Cross=Log | Circle=Kill");
        telemetry.addData("Turret Lock", turretLock ? "ON" : "OFF");

        telemetry.addLine("── POSITION ─────────────────────────────");
        telemetry.addData("X", String.format(Locale.US, "%.1f", px));
        telemetry.addData("Y", String.format(Locale.US, "%.1f", py));
        telemetry.addData("Heading", Pinpoint.INSTANCE.getHeading());

        telemetry.addLine("── SHOOTER ──────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP..." : "READY — press Cross to log");
        telemetry.addData("Velocity (actual)", (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)", (int) shootVelocity);
        telemetry.addData("Hood",              hoodPosition);
        telemetry.addData("Turret Angle",      Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Angle Set",         Turret.INSTANCE.getTurretAngleSet());

        telemetry.addLine("── SPINDEXER ────────────────────────────");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Ball 1", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball 2", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball 3", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Position", Spindexer.INSTANCE.getPosition());

        telemetry.addLine("── LOGGED POINTS ────────────────────────");
        telemetry.addData("Points logged", shooterLog.size());
        for (int i = 0; i < shooterLog.size(); i++) {
            telemetry.addData("S" + i, shooterLog.get(i));
            telemetry.addData("H" + i, hoodLog.get(i));
        }

        telemetry.update();

        lastSquare          = square;
        lastCross           = cross;
        lastTriangle        = triangle;
        lastCircle          = circle;
        lastLeftBumper      = leftBumper;
        lastRightBumper     = rightBumper;
        lastLeftTrigger     = leftTrigger;
        lastRightTriggerBtn = rightTrigger;
        lastDpadLeft        = dpad_left;
    }

    /** Returns first filled slot index, -1 if all empty. */
    private int filledPosition() {
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] != Spindexer.DetectedColor.EMPTY) return i;
        }
        return -1;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
    }
}