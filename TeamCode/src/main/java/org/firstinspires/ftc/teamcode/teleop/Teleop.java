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

    // ── Dashboard-tunable variables ───────────────────────────────────────────
    public static double shootVelocity = 3000; // tune this on the dashboard
    public static double turretAngle   = -185; // locked turret angle — tune on dashboard

    // ── Alliance ──────────────────────────────────────────────────────────────
    private Aliance currentAliance = Aliance.BLUE;

    // ── Mode flags ────────────────────────────────────────────────────────────
    private boolean shootMode    = false;
    private boolean turretLock   = false;
    private boolean transferDown = true;
    private boolean intakeOn     = false;

    // ── Square flywheel state machine ─────────────────────────────────────────
    // 0=off  1=spinning up  2=ready to shoot
    private int     squareState = 0;
    private boolean hasRumbled  = false;
    private int     patternStep = 0; // which shot in the pattern we're on (0/1/2)

    // ── Transfer flick state machine ──────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN, VERIFY }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();

    // ── Shoot verify dwell ────────────────────────────────────────────────────
    // After each flick, wait shootVerifySec then check if that slot is still
    // showing a ball (jammed). If so, seek back and re-flick automatically.
    public static double shootVerifySec = 0.3; // tunable on dashboard
    private int retrySlot = -1; // which slot we just tried to shoot

    // ── Spindexer intake dwell ────────────────────────────────────────────────
    public static double intakeDwellSec = 0.3; // tunable on dashboard
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;

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
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap); // calls start() internally
        MatchPattern.reset();

        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);

        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.addData("Alliance", currentAliance);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !lastDpadUp)     currentAliance = Aliance.BLUE;
        if (gamepad1.dpad_down && !lastDpadDown) currentAliance = Aliance.RED;

        lastDpadUp   = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        MatchPattern.tryDetect();

        telemetry.addLine("DPad UP = Blue  |  DPad DOWN = Red");
        telemetry.addData("Alliance selected", currentAliance);
        telemetry.addData("Pattern",           MatchPattern.getPattern());
        telemetry.addData("Pattern Locked",    MatchPattern.isLocked());
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
        boolean circle      = gamepad1.circle;
        boolean cross       = gamepad1.cross;
        boolean square      = gamepad1.square;
        boolean triangle    = gamepad1.triangle;
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        boolean dpadUp      = gamepad1.dpad_up;
        boolean dpadDown    = gamepad1.dpad_down;
        boolean dpadLeft    = gamepad1.dpad_left;
        boolean rightTrigger = gamepad1.right_trigger > 0.3;
        boolean leftTrigger  = gamepad1.left_trigger  > 0.3;

        // ── Triangle: switch to intake mode ───────────────────────────────────
        if (triangle && !lastTriangle) {
            intakeOn    = true;
            shootMode   = false;
            squareState = 0;
            hasRumbled  = false;
            patternStep = 0;
            dwelling    = false;
            Turret.INSTANCE.setVelocity(0);
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        // ── Circle: turn intake off OR switch to shoot mode ───────────────────
        if (circle && !lastCircle) {
            if (intakeOn) {
                intakeOn = false;
                dwelling = false;
                Intake.INSTANCE.idle();
            } else {
                shootMode = true;
                Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            }
        }

        // ── Cross: manual transfer flick ──────────────────────────────────────
        if (cross && !lastCross && flickState == FlickState.IDLE && transferDown) {
            transferDown = false;
            Transfer.INSTANCE.transferUp();
            flickTimer.reset();
            flickState = FlickState.WAIT_UP;
        }

        // ── Square: flywheel cycle ────────────────────────────────────────────
        if (square && !lastSquare) {
            switch (squareState) {
                case 0:
                    hasRumbled  = false;
                    patternStep = 0;
                    squareState = 1;
                    break;
                case 2:
                    if (flickState == FlickState.IDLE && transferDown) {
                        // Spindexer is already pre-positioned by the shoot mode
                        // routing block below — just fire immediately
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
                    patternStep = 0;
                    break;
            }
        }

        // ── DPad Up: turret lock toggle ───────────────────────────────────────
        if (dpadUp && !lastDpadUp) turretLock = !turretLock;

        // ── DPad Down: zero turret angle offset ───────────────────────────────
        if (dpadDown && !lastDpadDown) Turret.INSTANCE.zeroAngleOffset();

        // ── DPad Left: reverse intake ─────────────────────────────────────────
        if (dpadLeft && !lastDpadLeft) {
            if (intakeOn) Intake.INSTANCE.reverse();
            else          Intake.INSTANCE.idle();
        }

        // ── Triggers: fine-tune turret angle offset ───────────────────────────
        if (rightTrigger) Turret.INSTANCE.updateAngleOffset(-0.1);
        if (leftTrigger)  Turret.INSTANCE.updateAngleOffset(0.1);

        // ── Bumpers: manual spindexer rotation ────────────────────────────────
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
                    retrySlot = Spindexer.INSTANCE.getPosition().ordinal();
                    Spindexer.INSTANCE.setColor(
                            Spindexer.INSTANCE.getPosition(),
                            Spindexer.DetectedColor.EMPTY
                    );
                    transferDown = true;
                    flickTimer.reset();
                    flickState = FlickState.VERIFY;
                    // Advance pattern step after shot confirmed
                    if (shootMode && MatchPattern.isLocked()) {
                        patternStep++;
                        if (patternStep > 2) patternStep = 0;
                    }
                }
                break;
            case VERIFY:
                // Wait a moment then check if the slot still has a ball (jam)
                if (flickTimer.seconds() >= shootVerifySec) {
                    Spindexer.DetectedColor check = Spindexer.INSTANCE.readCurrentColor();
                    if (retrySlot != -1
                            && retrySlot == Spindexer.INSTANCE.getPosition().ordinal()
                            && (check == Spindexer.DetectedColor.GREEN
                            || check == Spindexer.DetectedColor.PURPLE)) {
                        // Ball still there — jammed. Re-stamp it and re-flick automatically.
                        Spindexer.INSTANCE.setColor(
                                Spindexer.INSTANCE.getPosition(), check
                        );
                        // Only retry if flywheel is still spinning
                        if (squareState > 0 && transferDown) {
                            transferDown = false;
                            Transfer.INSTANCE.transferUp();
                            flickTimer.reset();
                            flickState = FlickState.WAIT_UP;
                            // Don't increment patternStep — this is a retry
                        } else {
                            flickState = FlickState.IDLE;
                        }
                    } else {
                        // Clean shot confirmed
                        retrySlot  = -1;
                        flickState = FlickState.IDLE;
                    }
                }
                break;
            default:
                break;
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x
        );

        // ── Spindexer auto-rotate with dwell ──────────────────────────────────
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            if (intakeOn) {
                Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
                if (!dwelling) {
                    dwelling = true;
                    dwellTimer.reset();
                } else if (dwellTimer.seconds() >= intakeDwellSec) {
                    Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                    dwelling = false;
                    Spindexer.INSTANCE.periodic();
                    if (Spindexer.INSTANCE.getFull()) {
                        shootMode = true;
                        intakeOn  = false;
                        Intake.INSTANCE.idle();
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    } else {
                        int free = Spindexer.INSTANCE.freePosition();
                        if (free != -1) {
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                        }
                    }
                }
            } else {
                dwelling = false;
            }
        } else {
            // ── Shoot mode: continuously pre-position to the next pattern color ──
            dwelling = false;
            Spindexer.INSTANCE.periodic();
            if (transferDown) {
                if (Spindexer.INSTANCE.getEmpty()) {
                    shootMode   = false;
                    squareState = 0;
                    patternStep = 0;
                    Turret.INSTANCE.setVelocity(0);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                } else {
                    // Walk through pattern steps until we find a color we actually have
                    Spindexer.DetectedColor targetColor = null;
                    int targetPos = -1;
                    for (int offset = 0; offset < 3; offset++) {
                        int step = (patternStep + offset) % 3;
                        Spindexer.DetectedColor c = getTargetColor(step);
                        int pos = findColorPosition(c);
                        if (pos != -1) {
                            targetColor = c;
                            targetPos   = pos;
                            // Sync patternStep so shot order stays correct
                            patternStep = step;
                            break;
                        }
                    }

                    if (targetPos != -1) {
                        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[targetPos]);
                    } else {
                        // Pattern not locked or none match — fire whatever is available
                        int filled = Spindexer.INSTANCE.filledPosition();
                        if (filled != -1) {
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                        }
                    }
                }
            }
        }

        // ── Velocity & hood: Limelight distance when visible, else Pinpoint ────
        // shootVelocity is dashboard-tunable — used as flywheel speed when table
        // is not active (intake mode idle) and as a manual cap you can read on dash.
        int goalId = (currentAliance == Aliance.BLUE) ? Limelight.BLUE_GOAL_ID : Limelight.RED_GOAL_ID;
        double targetVelocity;
        double targetHood;
        double px = Pinpoint.INSTANCE.getPosX();
        double py = Pinpoint.INSTANCE.getPosY();

        if (!shootMode) {
            targetVelocity = 0;   // flywheel off during intake
            targetHood     = 1.0;
        } else {
            double llDist = Limelight.INSTANCE.distanceFromTag(goalId);
            if (llDist > 0) {
                targetVelocity = Turret.INSTANCE.distanceToVelocity(llDist, 0, currentAliance);
                targetHood     = Turret.INSTANCE.distanceToPosition(llDist, 0, currentAliance);
            } else {
                targetVelocity = Turret.INSTANCE.distanceToVelocity(px, py, currentAliance);
                targetHood     = Turret.INSTANCE.distanceToPosition(px, py, currentAliance);
            }
            // Dashboard override: if shootVelocity is manually set lower (e.g. for indoor
            // testing), cap targetVelocity to it so you never overshoot indoors
            if (shootVelocity > 0 && shootVelocity < targetVelocity) {
                targetVelocity = shootVelocity;
            }
        }

        // ── Turret aim: locked to turretAngle when not shooting ──────────────
        // turretAngle defaults to -185 to keep it out of the intake zone.
        // In shoot mode: Limelight angle if visible, else Pinpoint odometry.
        // turretLock forces park at turretAngle even during shoot mode.
        if (!shootMode || turretLock) {
            Turret.INSTANCE.setToAngle(turretAngle);
        } else {
            double llAngle = Limelight.INSTANCE.angleFromTag(goalId);
            if (llAngle != -1) {
                double corrected = Turret.INSTANCE.getTurretAngle() + llAngle;
                Turret.INSTANCE.setToAngle(corrected);
            } else {
                Turret.INSTANCE.followGoalOdometryPositional(currentAliance);
            }
        }

        if (squareState > 0) {
            Turret.INSTANCE.setVelocity(targetVelocity);
        } else {
            Turret.INSTANCE.setVelocity(0);
        }
        Turret.INSTANCE.setHoodPosition(targetHood);

        // ── Rumble: flywheel at speed ─────────────────────────────────────────
        if (squareState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - targetVelocity) < Turret.threshold
                && !hasRumbled) {
            gamepad1.rumbleBlips(1);
            hasRumbled  = true;
            squareState = 2;
        }

        // ── Rumble: ball detected (only during intake) ────────────────────────
        Spindexer.DetectedColor detected = intakeOn
                ? Spindexer.INSTANCE.readCurrentColor()
                : Spindexer.DetectedColor.EMPTY;
        if (detected == Spindexer.DetectedColor.GREEN) {
            gamepad1.rumble(1.0, 0.0, 200);
        } else if (detected == Spindexer.DetectedColor.PURPLE) {
            gamepad1.rumble(0.0, 1.0, 200);
        }

        // ── Periodic ─────────────────────────────────────────────────────────
        Turret.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        double llAngleTelem    = Limelight.INSTANCE.angleFromTag(goalId);
        double llDistanceTelem = Limelight.INSTANCE.distanceFromTag(goalId);
        boolean usingLL        = llDistanceTelem > 0;

        telemetry.addLine("── MATCH ────────────────────────────────");
        telemetry.addData("Alliance",         currentAliance);
        telemetry.addData("Pattern",          MatchPattern.getPattern()
                + (MatchPattern.isLocked() ? " (LOCKED)" : " (searching...)"));
        telemetry.addData("Mode",             shootMode ? "SHOOT" : "INTAKE");
        telemetry.addData("Intake",           intakeOn ? "ON" : "OFF");

        telemetry.addLine("── FLYWHEEL ─────────────────────────────");
        telemetry.addData("Flywheel",
                squareState == 0 ? "OFF" :
                        squareState == 1 ? "SPINNING UP..." : "READY - Press Square");
        telemetry.addData("Velocity (actual)",   (int) Turret.INSTANCE.getVelocity());
        telemetry.addData("Velocity (target)",   (int) targetVelocity);
        telemetry.addData("Velocity (dashboard)", (int) shootVelocity + " (cap)");
        telemetry.addData("Hood (target)",     targetHood);
        telemetry.addData("Shot #",            (patternStep + 1) + "/3");
        telemetry.addData("Next Expected",     MatchPattern.isLocked()
                ? String.valueOf(getTargetColor(patternStep)) : "ANY (no pattern)");

        telemetry.addLine("── TURRET ───────────────────────────────");
        telemetry.addData("Turret Lock",      turretLock ? "FORCED LOCK" : (shootMode ? "TRACKING" : "INTAKE LOCK"));
        telemetry.addData("Locked Angle",     turretAngle + " (dashboard)");
        telemetry.addData("Turret Angle",     Turret.INSTANCE.getTurretAngle());
        telemetry.addData("Turret Angle Set", Turret.INSTANCE.getTurretAngleSet());
        telemetry.addData("Aim Source",       (!shootMode || turretLock) ? "LOCKED" : (llAngleTelem != -1 ? "LIMELIGHT" : "PINPOINT"));

        telemetry.addLine("── LIMELIGHT ────────────────────────────");
        telemetry.addData("LL Angle",    llAngleTelem != -1 ? llAngleTelem + " deg" : "NOT FOUND");
        telemetry.addData("LL Distance", llDistanceTelem > 0 ? llDistanceTelem + " in" : "NOT FOUND");

        telemetry.addLine("── BILINEAR TABLE ───────────────────────");
        telemetry.addData("Distance Source", usingLL ? "LIMELIGHT" : "PINPOINT odometry");
        telemetry.addData("Distance Used",   usingLL
                ? llDistanceTelem + " in (LL)"
                : "(" + px + ", " + py + ") in (PP)");
        telemetry.addData("Table Velocity",  (int) targetVelocity + " ticks/s");
        telemetry.addData("Table Hood",      targetHood);

        telemetry.addLine("── ODOMETRY ─────────────────────────────");
        telemetry.addData("x",       px);
        telemetry.addData("y",       py);
        telemetry.addData("Heading", (((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360));

        telemetry.addLine("── SPINDEXER ────────────────────────────");
        telemetry.addData("Transfer Down",    transferDown);
        telemetry.addData("Flick State",      flickState);
        telemetry.addData("Retry Slot",       retrySlot != -1 ? "Slot " + retrySlot : "none");
        telemetry.addData("Dwell Active",     dwelling);
        telemetry.addData("Detected Color",   detected);
        telemetry.addData("Ball Pos 1",       Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball Pos 2",       Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball Pos 3",       Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Free Position",    Spindexer.INSTANCE.freePosition());
        telemetry.addData("Current Position", Spindexer.INSTANCE.getPosition());

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

    /**
     * Returns the expected color for the given pattern step (0/1/2).
     * Returns null if pattern is UNKNOWN or not locked.
     * GPP = Green, Purple, Purple
     * PGP = Purple, Green, Purple
     * PPG = Purple, Purple, Green
     */
    private Spindexer.DetectedColor getTargetColor(int step) {
        if (!MatchPattern.isLocked()) return null;
        step = step % 3;
        switch (MatchPattern.getPattern()) {
            case GPP: return step == 0 ? Spindexer.DetectedColor.GREEN : Spindexer.DetectedColor.PURPLE;
            case PGP: return step == 1 ? Spindexer.DetectedColor.GREEN : Spindexer.DetectedColor.PURPLE;
            case PPG: return step == 2 ? Spindexer.DetectedColor.GREEN : Spindexer.DetectedColor.PURPLE;
            default:  return null;
        }
    }

    /**
     * Finds the first slot index containing the given color, or -1 if not found.
     * Returns -1 if target is null or EMPTY.
     */
    private int findColorPosition(Spindexer.DetectedColor target) {
        if (target == null || target == Spindexer.DetectedColor.EMPTY) return -1;
        Spindexer.DetectedColor[] slots = Spindexer.INSTANCE.getBallAtPosition();
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == target) return i;
        }
        return -1;
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        Limelight.INSTANCE.stop();
    }
}