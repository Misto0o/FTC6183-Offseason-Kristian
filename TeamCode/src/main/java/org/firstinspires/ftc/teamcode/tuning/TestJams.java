package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Config
@TeleOp(name = "TestJams", group = "Tuning")
public class TestJams extends OpMode {

    // ── Tunables ──────────────────────────────────────────────────────────────
    public static double FLICK_UP_SEC   = 0.9;
    public static double FLICK_DOWN_SEC = 1.5;
    public static double SETTLE_SEC     = 0.3;
    public static double SHOOT_VELOCITY = 1075;

    // ── Flick state ───────────────────────────────────────────────────────────
    private enum FlickState { IDLE, WAIT_UP, WAIT_DOWN, WAIT_CLEAR }
    private FlickState flickState = FlickState.IDLE;
    private final ElapsedTime flickTimer = new ElapsedTime();
    private boolean transferDown = true;

    // ── Spindexer settle ──────────────────────────────────────────────────────
    private boolean spindexerSettled = true;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private int lastCommandedSlot = -1;

    // ── Stats ─────────────────────────────────────────────────────────────────
    private int totalShots   = 0;
    private int jamCount     = 0;
    private int currentSlot  = 0;  // which slot we're testing (0, 1, 2)
    private boolean flywheelOn = false;

    // ── Edge detection ────────────────────────────────────────────────────────
    private boolean prevSquare   = false;
    private boolean prevCross    = false;
    private boolean prevTriangle = false;
    private boolean prevCircle   = false;
    private boolean prevLB       = false;
    private boolean prevRB       = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDownAggressive();

        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColorSensor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColorSensor")
        );

        telemetry.addLine("TestSpindexer ready.");
        telemetry.addLine("□  = Fire current slot (holds on jam)");
        telemetry.addLine("✕  = Mark as JAMMED (count + skip)");
        telemetry.addLine("△  = Next slot manually");
        telemetry.addLine("○  = Reset fork down");
        telemetry.addLine("LB = Flywheel ON/OFF");
        telemetry.addLine("RB = Switch INTAKE / SHOOT mode");
        telemetry.update();
    }

    @Override
    public void start() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        commandSlot(0);
    }

    @Override
    public void loop() {
        Turret.INSTANCE.periodic();

        // ── Settle tracking ───────────────────────────────────────────────────
        if (!spindexerSettled && settleTimer.seconds() >= SETTLE_SEC) {
            spindexerSettled = true;
        }

        // ── Inputs ────────────────────────────────────────────────────────────
        boolean square   = gamepad1.square;
        boolean cross    = gamepad1.cross;
        boolean triangle = gamepad1.triangle;
        boolean circle   = gamepad1.circle;
        boolean lb       = gamepad1.left_bumper;
        boolean rb       = gamepad1.right_bumper;

        // ── LB: flywheel toggle ───────────────────────────────────────────────
        if (lb && !prevLB) {
            flywheelOn = !flywheelOn;
            Turret.INSTANCE.setVelocity(flywheelOn ? SHOOT_VELOCITY : 0);
            gamepad1.rumbleBlips(flywheelOn ? 2 : 1);
        }

        // ── Right trigger: re-send current slot position (for live tuning) ────────
        if (gamepad1.right_trigger > 0.3) {
            Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[currentSlot]);
        }

        // ── RB: toggle spindexer mode ─────────────────────────────────────────
        if (rb && !prevRB) {
            Spindexer.PositionType next = Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT
                    ? Spindexer.PositionType.INTAKE
                    : Spindexer.PositionType.SHOOT;
            Spindexer.INSTANCE.setPositionType(next);
            commandSlot(currentSlot);
            gamepad1.rumbleBlips(1);
        }

        // ── SQUARE: fire current slot ─────────────────────────────────────────
        // Holds in WAIT_CLEAR if it jams — doesn't auto-advance.
        if (square && !prevSquare && flickState == FlickState.IDLE && spindexerSettled && transferDown) {
            triggerFlick();
        }

        // ── CROSS: mark as jammed, skip to next slot ──────────────────────────
        if (cross && !prevCross) {
            jamCount++;
            resetFork();
            advanceSlot();
            gamepad1.rumble(0.5, 0.5, 300);
        }

        // ── TRIANGLE: manually advance to next slot ───────────────────────────
        if (triangle && !prevTriangle && flickState == FlickState.IDLE) {
            advanceSlot();
        }

        // ── CIRCLE: reset fork down ───────────────────────────────────────────
        if (circle && !prevCircle) {
            resetFork();
        }

        prevSquare = square; prevCross = cross;
        prevTriangle = triangle; prevCircle = circle;
        prevLB = lb; prevRB = rb;

        // ── Flick state machine ───────────────────────────────────────────────
        tickFlick();

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Flywheel",  flywheelOn
                ? String.format("ON  %d rpm", (int) Turret.INSTANCE.getVelocity())
                : "OFF");
        telemetry.addData("Mode",      Spindexer.INSTANCE.getPositionType());
        telemetry.addData("Slot",      String.format("%d  (settled: %s)", currentSlot + 1, spindexerSettled ? "✓" : "..."));
        telemetry.addData("Fork",      flickState + (transferDown ? " (down)" : " (up)"));
        telemetry.addLine();
        telemetry.addData("Total fired", totalShots);
        telemetry.addData("Jams",        jamCount);
        telemetry.addLine();
        telemetry.addLine("□ Fire  ✕ Jam  △ Next  ○ Reset fork  LB Flywheel  RB Mode");
        telemetry.update();
    }

    private void triggerFlick() {
        transferDown = false;
        Transfer.INSTANCE.transferUpAggressive();
        flickTimer.reset();
        flickState = FlickState.WAIT_UP;
    }

    private void resetFork() {
        Transfer.INSTANCE.transferDownAggressive();
        flickState   = FlickState.IDLE;
        transferDown = true;
    }

    private void advanceSlot() {
        currentSlot = (currentSlot + 1) % 3;
        commandSlot(currentSlot);
    }

    private void commandSlot(int slot) {
        if (slot != lastCommandedSlot) {
            lastCommandedSlot  = slot;
            spindexerSettled   = false;
            settleTimer.reset();
        }
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[slot]);
    }

    private void tickFlick() {
        switch (flickState) {
            case WAIT_UP:
                Transfer.INSTANCE.transferUpAggressive();
                if (flickTimer.seconds() >= FLICK_UP_SEC) {
                    Transfer.INSTANCE.transferDownAggressive();
                    flickTimer.reset();
                    flickState = FlickState.WAIT_DOWN;
                }
                break;

            case WAIT_DOWN:
                if (flickTimer.seconds() >= FLICK_DOWN_SEC) {
                    // Fork returned — check if ball actually cleared
                    // If you have a distance sensor you could check here
                    // For now just mark success and wait for driver confirmation
                    totalShots++;
                    transferDown = true;
                    flickState   = FlickState.IDLE;
                    // Don't auto-advance — driver decides with △ or □ again
                }
                break;

            default:
                break;
        }
    }

    @Override
    public void stop() {
        Turret.INSTANCE.setVelocity(0);
        Transfer.INSTANCE.transferDownAggressive();
    }
}