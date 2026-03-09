package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Transfer;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Utils.MatchPattern;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

@Config
@Autonomous(name = "Red Close Six Ball Auto", group = "Red")
public class RedCloseSixBallAuto extends LinearOpMode {

    // ── Tunable delays (ms) ───────────────────────────────────────────────────
    public static long TRANSFER_FLICK_UP_MS   = 500;
    public static long TRANSFER_FLICK_DOWN_MS = 400;
    public static long SPINDEXER_SETTLE_MS    = 600;
    public static long AT_VELOCITY_TIMEOUT_MS = 3000;

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Follower follower;
    private Path p1, p2, p3, p4;

    // ── Starting pose ─────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(116.816, 130.041, Math.toRadians(38));

    @Override
    public void runOpMode() {
        // ── Init hardware ─────────────────────────────────────────────────────
        follower = Constants.createFollower(hardwareMap);

        Intake.INSTANCE.init(hardwareMap);
        Spindexer.INSTANCE.initialize(
                hardwareMap.servo.get("spinServo"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "leftColor"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, "rightColor")
        );
        Transfer.INSTANCE.initialize(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
        MatchPattern.reset();

        buildPaths();

        // ── Wait for start — detect pattern while waiting ─────────────────────
        while (!isStarted() && !isStopRequested()) {
            MatchPattern.tryDetect();
            telemetry.addLine("Ready — Red Close Six Ball Auto");
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Pattern", MatchPattern.getPattern());
            telemetry.addData("Pattern Locked", MatchPattern.isLocked());
            telemetry.update();
        }
        if (!opModeIsActive()) return;

        // ── Set starting pose ─────────────────────────────────────────────────
        follower.setStartingPose(startPose);
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 116.816, 130.041, AngleUnit.DEGREES, 38)
        );

        // ── Spin up flywheel immediately ──────────────────────────────────────
        Turret.INSTANCE.setVelocity(
                Turret.INSTANCE.distanceToVelocity(startPose.getX(), startPose.getY(), Aliance.RED)
        );
        Turret.INSTANCE.setHoodPosition(
                Turret.INSTANCE.distanceToPosition(startPose.getX(), startPose.getY(), Aliance.RED)
        );

        // ── Drive to shoot position ───────────────────────────────────────────
        followPath(p1);

        // ── Shoot first three ─────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake mode → curve to row 1, sweep through, return ──────────────
        intakeMode();
        followPath(p2);
        followPath(p3);
        followPath(p4);

        // ── Shoot second three ────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // NOTE: Cheick's original called shootThree() twice here (same as NineBall).
        // Likely a mistake — second shootThree() commented out until row 2 path exists.
        // shootThree();

        // ── Done ──────────────────────────────────────────────────────────────
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shoot sequence — waits for velocity, then flicks all 3 positions
    // ─────────────────────────────────────────────────────────────────────────
    private void shootThree() {
        Intake.INSTANCE.idle();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.RED));
        Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.RED));

        waitForVelocity();

        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);
        tickPeriodicFor(SPINDEXER_SETTLE_MS);
        flick(Spindexer.Position.POSITION_ONE);

        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_TWO);
        tickPeriodicFor(SPINDEXER_SETTLE_MS);
        flick(Spindexer.Position.POSITION_TWO);

        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_THREE);
        tickPeriodicFor(SPINDEXER_SETTLE_MS);
        flick(Spindexer.Position.POSITION_THREE);
    }

    /** Transfer up → wait → down → mark empty */
    private void flick(Spindexer.Position position) {
        Transfer.INSTANCE.transferUp();
        sleep(TRANSFER_FLICK_UP_MS);
        Transfer.INSTANCE.transferDown();
        sleep(TRANSFER_FLICK_DOWN_MS);
        Spindexer.INSTANCE.setColor(position, Spindexer.DetectedColor.EMPTY);
    }

    /** Switch to intake mode and spin up intake */
    private void intakeMode() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Intake.INSTANCE.on();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Path following — blocks until Pedro reaches the end of the path
    // ─────────────────────────────────────────────────────────────────────────
    private void followPath(Path path) {
        follower.followPath(path, true);
        while (opModeIsActive() && follower.isBusy()) {
            updatePeriodicSystems();
            telemetry.addData("Following path", "busy");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }

    /** Poll isAtVelocity() until true or timeout */
    private void waitForVelocity() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()
                && !Turret.INSTANCE.isAtVelocity()
                && timer.milliseconds() < AT_VELOCITY_TIMEOUT_MS) {
            Turret.INSTANCE.periodic();
            Spindexer.INSTANCE.periodic();
            Pinpoint.INSTANCE.periodic();
        }
    }

    /** Run periodic systems for a set duration in ms */
    private void tickPeriodicFor(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            updatePeriodicSystems();
        }
    }

    /** Keep all subsystems ticking every loop */
    private void updatePeriodicSystems() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT) {
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.RED);
            Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.RED));
            Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.RED));
        } else {
            Turret.INSTANCE.setToAngle(270);
            Turret.INSTANCE.setVelocity(500); // warm during intake driving
        }

        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            int free = Spindexer.INSTANCE.freePosition();
            if (free != -1) {
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[free]);
            }
        }

        follower.update();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();
        Pinpoint.INSTANCE.periodic();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Paths — coordinates from Cheick's original RedCloseSixBallAuto
    // ─────────────────────────────────────────────────────────────────────────
    private void buildPaths() {
        // p1: start → shoot position (mirror of blue p1)
        p1 = new Path(new BezierLine(
                new Pose(116.816, 130.041),
                new Pose(82.285, 94.224)
        ));
        p1.setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0));

        // p2: shoot position → curve to start of row 1
        p2 = new Path(new BezierCurve(
                new Pose(82.285, 94.224),
                new Pose(80.449, 54.735),
                new Pose(107.796, 59.714)
        ));
        p2.setConstantHeadingInterpolation(Math.toRadians(0));

        // p3: sweep through row 1 balls
        p3 = new Path(new BezierLine(
                new Pose(107.796, 59.714),
                new Pose(122.367, 59.755)
        ));
        p3.setConstantHeadingInterpolation(Math.toRadians(0));

        // p4: return to shoot position
        p4 = new Path(new BezierLine(
                new Pose(122.367, 59.755),
                new Pose(82.163, 93.878)
        ));
        p4.setConstantHeadingInterpolation(Math.toRadians(0));
    }
}