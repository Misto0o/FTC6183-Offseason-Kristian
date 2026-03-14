package org.firstinspires.ftc.teamcode.Outreach;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "Outreach", group = "!")
@Config
public class Outreach extends OpMode {

    // ── Dashboard-tunable variables ───────────────────────────────────────────
    public static double shootVelocity = 3000;
    // Spindexer positions are tuned directly via Spindexer's @Config on the dashboard

    // ── Shooter state machine ─────────────────────────────────────────────────
    // 0 = off
    // 1 = spinning up (waiting for velo)
    // 2 = ready — square fires a ball, re-arms to 1 after each shot
    // 3 = spinning down (square pressed after all balls shot, or manually)
    private int shooterState = 0;
    private boolean hasRumbled = false;
    private int ballsFired = 0; // track how many balls fired in manual sequence

    // ── Shoot-three state machine (Cross — auto) ──────────────────────────────
    private enum ShootCycleState {
        IDLE,
        SET_POS, WAIT_POS,
        TRANSFER_UP,
        TRANSFER_DOWN,
        MARK_EMPTY
    }
    private ShootCycleState shootCycleState = ShootCycleState.IDLE;
    private int shootBallIndex = 0;
    private boolean shootCycleActive = false;
    private static final Spindexer.Position[] SHOOT_ORDER = {
            Spindexer.Position.POSITION_ONE,
            Spindexer.Position.POSITION_TWO,
            Spindexer.Position.POSITION_THREE
    };
    private final ElapsedTime shootCycleTimer = new ElapsedTime();

    // ── Transfer flick state machine (manual single shot) ─────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();
    private boolean transferDown = true;

    // ── Intake dwell timer (pause at each slot before advancing) ─────────────
    public static double intakeDwellSec = 0.3; // tunable on dashboard
    private final ElapsedTime dwellTimer = new ElapsedTime();
    private boolean dwelling = false;

    // ── Intake toggle ─────────────────────────────────────────────────────────
    private boolean intakeOn = false;

    // ── Button edge detection ─────────────────────────────────────────────────
    private boolean lastSquare, lastTriangle, lastCircle, lastCross;
    private boolean lastDpadLeft, lastDpadUp, lastDpadDown;
    private boolean lastLeftBumper, lastRightBumper;

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

        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Turret.INSTANCE.setToAngle(-90);

        telemetry.addLine("Welcome to FTC 6183 Loki!");
        telemetry.addLine("Triangle=Intake | Square=Rev/Shoot/Off | Cross=Auto Shoot All");
        telemetry.addLine("DPad Left=Kill | DPad Up=3000 | DPad Down=1000");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean square      = gamepad1.square;
        boolean triangle    = gamepad1.triangle;
        boolean circle      = gamepad1.circle;
        boolean cross       = gamepad1.cross;
        boolean dpadLeft    = gamepad1.dpad_left;
        boolean dpadUp      = gamepad1.dpad_up;
        boolean dpadDown    = gamepad1.dpad_down;
        boolean leftBumper  = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        // ── Turret locked at -90 always ───────────────────────────────────────
        Turret.INSTANCE.setToAngle(-90);

        // ── DPad Left: kill everything ────────────────────────────────────────
        if (dpadLeft && !lastDpadLeft) killEverything();

        // ── DPad Up: set velo to 3000 ─────────────────────────────────────────
        if (dpadUp && !lastDpadUp) {
            shootVelocity = 3000;
            if (shooterState >= 1 && shooterState <= 2) {
                Turret.INSTANCE.setVelocity(shootVelocity);
            }
        }

        // ── DPad Down: set velo to 1000 ───────────────────────────────────────
        if (dpadDown && !lastDpadDown) {
            shootVelocity = 1000;
            if (shooterState >= 1 && shooterState <= 2) {
                Turret.INSTANCE.setVelocity(shootVelocity);
            }
        }

        // ── Triangle: switch to intake mode ───────────────────────────────────
        if (triangle && !lastTriangle && !shootCycleActive) {
            intakeOn     = true;
            shooterState = 0;
            hasRumbled   = false;
            ballsFired   = 0;
            Turret.INSTANCE.setVelocity(0);
            Intake.INSTANCE.on();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        }

        // ── Circle: switch to shooter mode / spin up ──────────────────────────
        if (circle && !lastCircle && !shootCycleActive) {
            intakeOn     = false;
            hasRumbled   = false;
            ballsFired   = 0;
            Intake.INSTANCE.idle();
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            shooterState = 1;
            Turret.INSTANCE.setVelocity(shootVelocity);
        }

        // ── Square: rev → shoot x3 → off ─────────────────────────────────────
        // State 0: first press → spin up
        // State 2: press → fire one ball, re-arms to state 1 (waits for velo again)
        //          after 3 balls fired, next press turns off
        // State 1: press while spinning up → cancel (kill)
        if (square && !lastSquare && !shootCycleActive) {
            switch (shooterState) {
                case 0:
                    // Rev up
                    shooterState = 1;
                    hasRumbled   = false;
                    ballsFired   = 0;
                    intakeOn     = false;
                    Intake.INSTANCE.idle();
                    Turret.INSTANCE.setVelocity(shootVelocity);
                    Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                    break;
                case 2:
                    if (ballsFired >= 3) {
                        // All 3 fired — turn off shooter
                        killEverything();
                    } else if (flickState == FlickState.IDLE && transferDown) {
                        // Fire one ball
                        transferDown = false;
                        Transfer.INSTANCE.transferUp();
                        flickTimer.reset();
                        flickState   = FlickState.WAIT_UP;
                        ballsFired++;
                        shooterState = 1; // re-arm: wait for velo before next shot
                        hasRumbled   = false;
                    }
                    break;
                case 1:
                default:
                    // Cancel during spin-up
                    killEverything();
                    break;
            }
        }

        // ── Cross: auto shoot-all-three cycle ─────────────────────────────────
        if (cross && !lastCross && shootCycleState == ShootCycleState.IDLE && !shootCycleActive) {
            shootBallIndex   = 0;
            shootCycleActive = true;
            intakeOn         = false;
            Intake.INSTANCE.idle();
            Turret.INSTANCE.setVelocity(shootVelocity);
            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
            shootCycleState = ShootCycleState.SET_POS;
            shootCycleTimer.reset();
        }

        // ── Bumpers: manual spindexer rotation ────────────────────────────────
        if (!shootCycleActive) {
            if (leftBumper && !lastLeftBumper) {
                Spindexer.Position.next();
                Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
            }
            if (rightBumper && !lastRightBumper) {
                Spindexer.Position.previous();
                Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition());
            }
        }

        // ── Manual flick state machine ────────────────────────────────────────
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

        // ── Auto shoot-all-three state machine ────────────────────────────────
        switch (shootCycleState) {
            case SET_POS:
                Spindexer.INSTANCE.setToPosition(SHOOT_ORDER[shootBallIndex]);
                shootCycleTimer.reset();
                shootCycleState = ShootCycleState.WAIT_POS;
                break;
            case WAIT_POS:
                if (shootCycleTimer.seconds() >= 0.5) {
                    Transfer.INSTANCE.transferUp();
                    shootCycleTimer.reset();
                    shootCycleState = ShootCycleState.TRANSFER_UP;
                }
                break;
            case TRANSFER_UP:
                if (shootCycleTimer.seconds() >= 0.5) {
                    Transfer.INSTANCE.transferDown();
                    shootCycleTimer.reset();
                    shootCycleState = ShootCycleState.TRANSFER_DOWN;
                }
                break;
            case TRANSFER_DOWN:
                if (shootCycleTimer.seconds() >= 0.5) {
                    shootCycleState = ShootCycleState.MARK_EMPTY;
                }
                break;
            case MARK_EMPTY:
                Spindexer.INSTANCE.setColor(
                        Spindexer.INSTANCE.getPosition(),
                        Spindexer.DetectedColor.EMPTY
                );
                shootBallIndex++;
                if (shootBallIndex >= 3) {
                    shootCycleActive = false;
                    shootCycleState  = ShootCycleState.IDLE;
                    killEverything();
                } else {
                    shootCycleState = ShootCycleState.SET_POS;
                }
                break;
            default:
                break;
        }

        // ── Spindexer auto-rotate + mode switching ────────────────────────────
        if (!shootCycleActive && Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT) {
            Spindexer.INSTANCE.periodic(); // keep full/empty flags fresh in shoot mode
        }

        if (!shootCycleActive) {
            if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
                if (intakeOn) {
                    // Read color and dwell at current slot before deciding to advance
                    Spindexer.DetectedColor seen = Spindexer.INSTANCE.readCurrentColor();
                    int currentIdx = Spindexer.INSTANCE.getPosition().ordinal();

                    if (!dwelling) {
                        // Start dwell — stay here so sensor can confirm what it sees
                        dwelling = true;
                        dwellTimer.reset();
                    } else if (dwellTimer.seconds() >= intakeDwellSec) {
                        // Dwell complete — stamp the confirmed color then advance
                        Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), seen);
                        dwelling = false;
                        Spindexer.INSTANCE.periodic(); // recompute full/empty after stamp

                        if (Spindexer.INSTANCE.getFull()) {
                            Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
                            intakeOn = false;
                            Intake.INSTANCE.idle();
                            int filled = Spindexer.INSTANCE.filledPosition();
                            if (filled != -1) {
                                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                            }
                        } else {
                            int free = Spindexer.INSTANCE.freePosition();
                            if (free != -1) {
                                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
                            }
                        }
                    }
                    // While dwelling: don't move, just keep reading
                } else {
                    dwelling = false;
                }
            } else {
                // Shoot mode — sit on next filled slot while transfer is down
                dwelling = false;
                if (transferDown) {
                    if (Spindexer.INSTANCE.getEmpty()) {
                        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
                        shooterState = 0;
                        Turret.INSTANCE.setVelocity(0);
                    } else {
                        int filled = Spindexer.INSTANCE.filledPosition();
                        if (filled != -1) {
                            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[filled]);
                        }
                    }
                }
            }
        }

        // ── Flywheel ready check + rumble ─────────────────────────────────────
        if (shooterState == 1
                && Math.abs(Turret.INSTANCE.getVelocity() - shootVelocity) < Turret.threshold) {
            shooterState = 2;
            if (!hasRumbled) {
                gamepad1.rumbleBlips(3);
                hasRumbled = true;
            }
        }

        // ── Drivetrain ────────────────────────────────────────────────────────
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x
        );

        // ── Periodic ─────────────────────────────────────────────────────────
        Turret.INSTANCE.periodic();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addLine("Welcome to FTC 6183 Loki!");
        telemetry.addData("Intake",       intakeOn ? "ON" : "OFF");
        telemetry.addData("Flywheel",
                shooterState == 0 ? "OFF" :
                        shooterState == 1 ? "SPINNING UP..." :
                                "READY - Press Square! (" + ballsFired + "/3 fired)");
        telemetry.addData("Velo Target",  shootVelocity);
        telemetry.addData("Shoot Cycle",  shootCycleActive ? ("Ball " + (shootBallIndex + 1) + "/3") : "IDLE");
        telemetry.addData("Mode",         Spindexer.INSTANCE.getPositionType());
        telemetry.addData("Position",     Spindexer.INSTANCE.getPosition());
        telemetry.addData("Ball 1",       Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball 2",       Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball 3",       Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.update();

        // ── Save button state ─────────────────────────────────────────────────
        lastSquare      = square;
        lastTriangle    = triangle;
        lastCircle      = circle;
        lastCross       = cross;
        lastDpadLeft    = dpadLeft;
        lastDpadUp      = dpadUp;
        lastDpadDown    = dpadDown;
        lastLeftBumper  = leftBumper;
        lastRightBumper = rightBumper;
    }

    @Override
    public void stop() {
        killEverything();
    }

    private void killEverything() {
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
        Transfer.INSTANCE.transferDown();
        shooterState     = 0;
        shootCycleActive = false;
        shootCycleState  = ShootCycleState.IDLE;
        flickState       = FlickState.IDLE;
        hasRumbled       = false;
        ballsFired       = 0;
        intakeOn         = false;
        dwelling         = false;
        transferDown     = true;
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
    }
}