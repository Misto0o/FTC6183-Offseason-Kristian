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
@Autonomous(name = "Blue Close Nine Ball Auto", group = "Blue")
public class BlueCloseNineBallAuto extends LinearOpMode {

    // ── Tunable delays (ms) ───────────────────────────────────────────────────
    public static long TRANSFER_FLICK_UP_MS   = 500;
    public static long TRANSFER_FLICK_DOWN_MS = 400;
    public static long SPINDEXER_SETTLE_MS    = 600;
    public static long AT_VELOCITY_TIMEOUT_MS = 3000;

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Follower follower;
    private Path p1, p2, p3, p4, p5;

    // ── Starting pose ─────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(27.184, 130.041, Math.toRadians(142));

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
            telemetry.addLine("Ready — Blue Close Nine Ball Auto");
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Pattern", MatchPattern.getPattern());
            telemetry.addData("Pattern Locked", MatchPattern.isLocked());
            telemetry.update();
        }
        if (!opModeIsActive()) return;

        // ── Set starting pose ─────────────────────────────────────────────────
        follower.setStartingPose(startPose);
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 27.184, 130.041, AngleUnit.DEGREES, 142)
        );

        // ── Spin up flywheel immediately ──────────────────────────────────────
        Turret.INSTANCE.setVelocity(
                Turret.INSTANCE.distanceToVelocity(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );
        Turret.INSTANCE.setHoodPosition(
                Turret.INSTANCE.distanceToPosition(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );

        // ── Drive to shoot position ───────────────────────────────────────────
        followPath(p1);

        // ── Shoot first three ─────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake mode → curve to row 1, sweep through balls, return ─────────
        intakeMode();
        followPath(p2); // curve to start of row 1
        followPath(p3); // sweep through balls (0.75 speed in Cheick's original)
        followPath(p4); // return to shoot position

        // ── Shoot second three ────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake mode → sweep row 2 ─────────────────────────────────────────
        // NOTE: Cheick's original had p5 commented out and called shootThree() twice.
        // Keeping that structure — p5 is built but unused until path is confirmed on field.
        intakeMode();
        // followPath(p5); // uncomment once path is verified

        // ── Shoot third three (from same position, second spindexer load) ─────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

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
        Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.BLUE));
        Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.BLUE));

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
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.BLUE));
            Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.BLUE));
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
    // Paths — coordinates from Cheick's original BlueCloseNineBallAuto
    // ─────────────────────────────────────────────────────────────────────────
    private void buildPaths() {
        // p1: start → shoot position
        p1 = new Path(new BezierLine(
                new Pose(27.184, 130.041),
                new Pose(61.531, 94.224)
        ));
        p1.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // p2: shoot position → curve to start of row 1
        p2 = new Path(new BezierCurve(
                new Pose(61.531, 94.224),
                new Pose(63.551, 54.735),
                new Pose(36.204, 59.714)
        ));
        p2.setConstantHeadingInterpolation(Math.toRadians(180));

        // p3: sweep through row 1 balls
        p3 = new Path(new BezierLine(
                new Pose(36.204, 59.714),
                new Pose(21.633, 59.755)
        ));
        p3.setConstantHeadingInterpolation(Math.toRadians(180));

        // p4: return to shoot position
        p4 = new Path(new BezierLine(
                new Pose(21.633, 59.755),
                new Pose(61.837, 93.878)
        ));
        p4.setConstantHeadingInterpolation(Math.toRadians(180));

        // p5: row 2 intake sweep (currently commented out in autonomousRoutine — verify on field)
        p5 = new Path(new BezierLine(
                new Pose(22.061, 84.143),
                new Pose(61.612, 85.122)
        ));
        p5.setConstantHeadingInterpolation(Math.toRadians(180));
    }
}