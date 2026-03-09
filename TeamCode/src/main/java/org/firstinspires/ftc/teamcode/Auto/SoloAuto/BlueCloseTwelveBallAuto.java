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
@Autonomous(name = "Blue Close Twelve Ball Auto", group = "Blue")
public class BlueCloseTwelveBallAuto extends LinearOpMode {

    // ── Tunable delays (ms) ───────────────────────────────────────────────────
    public static long TRANSFER_FLICK_UP_MS   = 500;
    public static long TRANSFER_FLICK_DOWN_MS = 400;
    public static long SPINDEXER_SETTLE_MS    = 600;
    public static long AT_VELOCITY_TIMEOUT_MS = 3000;

    // ── Paths ─────────────────────────────────────────────────────────────────
    // NOTE: Cheick's original had everything after returnOne commented out.
    // All paths are built and available — uncomment followPath() calls below
    // as each path is verified on the field.
    private Follower follower;
    private Path pickUpFirstRow, returnOne;
    private Path pickUpSecondRow, gateOne, gateTwo, returnTwo;
    private Path pickUpThirdRow, end;

    // ── Starting pose ─────────────────────────────────────────────────────────
    private final Pose startPose = new Pose(24.980, 127.469, Math.toRadians(142));

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
            telemetry.addLine("Ready — Blue Close Twelve Ball Auto");
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Pattern", MatchPattern.getPattern());
            telemetry.addData("Pattern Locked", MatchPattern.isLocked());
            telemetry.update();
        }
        if (!opModeIsActive()) return;

        // ── Set starting pose ─────────────────────────────────────────────────
        follower.setStartingPose(startPose);
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 24.980, 127.469, AngleUnit.DEGREES, 142)
        );

        // ── Spin up flywheel immediately ──────────────────────────────────────
        Turret.INSTANCE.setVelocity(
                Turret.INSTANCE.distanceToVelocity(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );
        Turret.INSTANCE.setHoodPosition(
                Turret.INSTANCE.distanceToPosition(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );

        // ── Shoot preload (Cheick's original: waitToShoot → shootThree from start) ──
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        waitForVelocity();
        shootThree();

        // ── Intake row 1 ──────────────────────────────────────────────────────
        intakeMode();
        followPath(pickUpFirstRow);
        followPath(returnOne);

        // ── Shoot second three ────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake row 2 (commented out in Cheick's original — verify paths) ──
        // intakeMode();
        // followPath(pickUpSecondRow);
        // followPath(gateOne);
        // followPath(gateTwo);
        // followPath(returnTwo);
        // Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        // shootThree();

        // ── Intake row 3 (commented out in Cheick's original — verify paths) ──
        // intakeMode();
        // followPath(pickUpThirdRow);
        // Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        // shootThree();
        // followPath(end);

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
    // Paths — coordinates from Cheick's original BlueCloseTwelveBallAuto
    // ─────────────────────────────────────────────────────────────────────────
    private void buildPaths() {
        // Row 1 intake — curve from start directly into row 1
        pickUpFirstRow = new Path(new BezierCurve(
                new Pose(24.980, 127.469),
                new Pose(73.112, 78.684),
                new Pose(16.714, 83.755)
        ));
        pickUpFirstRow.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // Return from row 1 to shoot position
        returnOne = new Path(new BezierCurve(
                new Pose(16.714, 83.755),
                new Pose(62.061, 83.816)
        ));
        returnOne.setConstantHeadingInterpolation(Math.toRadians(180));

        // Row 2 intake — curve from shoot position into row 2
        pickUpSecondRow = new Path(new BezierCurve(
                new Pose(62.061, 83.816),
                new Pose(61.306, 57.449),
                new Pose(16.061, 59.735)
        ));
        pickUpSecondRow.setTangentHeadingInterpolation();

        // Gate maneuver 1
        gateOne = new Path(new BezierLine(
                new Pose(16.061, 59.735),
                new Pose(16.224, 71.449)
        ));
        gateOne.setTangentHeadingInterpolation();

        // Gate maneuver 2
        gateTwo = new Path(new BezierLine(
                new Pose(16.224, 71.449),
                new Pose(15.347, 70.959)
        ));
        gateTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        // Return from row 2 / gate area
        returnTwo = new Path(new BezierLine(
                new Pose(15.347, 70.959),
                new Pose(73.020, 71.510)
        ));
        returnTwo.setTangentHeadingInterpolation();

        // Row 3 intake — curve from return into row 3
        pickUpThirdRow = new Path(new BezierCurve(
                new Pose(73.020, 71.510),
                new Pose(75.306, 35.633),
                new Pose(13.367, 34.939)
        ));
        pickUpThirdRow.setTangentHeadingInterpolation();

        // End — return from row 3 to shoot/park position
        end = new Path(new BezierLine(
                new Pose(13.367, 34.939),
                new Pose(72.918, 71.449)
        ));
        end.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142));
    }
}