package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
@Autonomous(name = "Blue Far Nine Ball Auto", group = "Blue")
public class Move1tile extends LinearOpMode {

    // ── Tunable delays (ms) ───────────────────────────────────────────────────
    public static long TRANSFER_FLICK_UP_MS   = 500;
    public static long TRANSFER_FLICK_DOWN_MS = 400;
    public static long SPINDEXER_SETTLE_MS    = 600;
    public static long AT_VELOCITY_TIMEOUT_MS = 3000;

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Follower follower;

    // Start → far shoot zone
    private PathChain toShootZone;
    // Shoot zone → row 1 balls (far row)
    private PathChain toRow1;
    // Row 1 → shoot zone
    private PathChain returnFromRow1;
    // Shoot zone → row 2 balls (close row)
    private PathChain toRow2;
    // Row 2 → shoot zone
    private PathChain returnFromRow2;
    // Shoot zone → park
    private PathChain toPark;

    // ── Starting pose ─────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

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
            telemetry.addLine("Ready — Blue Far Nine Ball Auto");
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Pattern", MatchPattern.getPattern());
            telemetry.addData("Pattern Locked", MatchPattern.isLocked());
            telemetry.update();
        }
        if (!opModeIsActive()) return;

        // ── Set starting pose ─────────────────────────────────────────────────
        follower.setStartingPose(startPose);
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 56.000, 8.000, AngleUnit.DEGREES, 90)
        );

        // ── Spin up flywheel immediately ──────────────────────────────────────
        Turret.INSTANCE.setVelocity(
                Turret.INSTANCE.distanceToVelocity(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );
        Turret.INSTANCE.setHoodPosition(
                Turret.INSTANCE.distanceToPosition(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );

        // ── Drive to far shoot zone and shoot preload ─────────────────────────
        followPath(toShootZone);
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake row 1 (far row) ────────────────────────────────────────────
        intakeMode();
        followPath(toRow1);
        followPath(returnFromRow1);

        // ── Shoot second three ────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake row 2 (close row) ──────────────────────────────────────────
        intakeMode();
        followPath(toRow2);
        followPath(returnFromRow2);

        // ── Shoot third three ─────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Park ──────────────────────────────────────────────────────────────
        followPath(toPark);

        // ── Done ──────────────────────────────────────────────────────────────
        Turret.INSTANCE.setVelocity(0);
        Intake.INSTANCE.idle();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shoot sequence
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

    private void flick(Spindexer.Position position) {
        Transfer.INSTANCE.transferUp();
        sleep(TRANSFER_FLICK_UP_MS);
        Transfer.INSTANCE.transferDown();
        sleep(TRANSFER_FLICK_DOWN_MS);
        Spindexer.INSTANCE.setColor(position, Spindexer.DetectedColor.EMPTY);
    }

    private void intakeMode() {
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE);
        Intake.INSTANCE.on();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Path following
    // ─────────────────────────────────────────────────────────────────────────
    private void followPath(PathChain path) {
        follower.followPath(path, true);
        while (opModeIsActive() && follower.isBusy()) {
            updatePeriodicSystems();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }

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

    private void tickPeriodicFor(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            updatePeriodicSystems();
        }
    }

    private void updatePeriodicSystems() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT) {
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.BLUE));
            Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.BLUE));
        } else {
            Turret.INSTANCE.setToAngle(270);
            Turret.INSTANCE.setVelocity(500);
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
    // Paths — from your PP layout, cleaned up and renamed
    // ─────────────────────────────────────────────────────────────────────────
    private void buildPaths() {
        // Start (56, 8) → far shoot zone (56, 36), rotate to face goal
        toShootZone = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.000, 36.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        // Shoot zone → row 1 balls (far row at y≈35)
        toRow1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 36.000),
                        new Pose(8.462, 35.746)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Row 1 → shoot zone
        returnFromRow1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(8.462, 35.746),
                        new Pose(67.867, 85.618)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Shoot zone → row 2 balls (close row at y≈83)
        toRow2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(67.867, 85.618),
                        new Pose(14.428, 83.792)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Row 2 → shoot zone (y≈119, close to goal)
        returnFromRow2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(14.428, 83.792),
                        new Pose(52.301, 119.607)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Park
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(52.301, 119.607),
                        new Pose(21.468, 72.445)
                ))
                .setTangentHeadingInterpolation()
                .build();
    }
}