package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
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

@Config
@Autonomous(name = "Blue Close Six Ball Auto", group = "Blue")
public class BlueCloseSixBallAuto extends LinearOpMode {

    // ── Tunable delays (ms) ───────────────────────────────────────────────────
    public static long TRANSFER_FLICK_UP_MS   = 500;
    public static long TRANSFER_FLICK_DOWN_MS = 400;
    public static long SPINDEXER_SETTLE_MS    = 600;
    public static long AT_VELOCITY_TIMEOUT_MS = 3000; // max wait for flywheel

    // ── Paths ─────────────────────────────────────────────────────────────────
    private Follower follower;
    private Path p1, p2, p3, p4, p5, p6, p7;

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

        buildPaths();

        telemetry.addLine("Ready — Blue Close Six Ball Auto");
        telemetry.addData("Alliance", "BLUE");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ── Set starting pose ─────────────────────────────────────────────────
        follower.setStartingPose(startPose);
        Pinpoint.INSTANCE.updatePosition(
                new Pose2D(DistanceUnit.INCH, 27.184, 130.041, AngleUnit.DEGREES, 142)
        );

        // ── Spin up flywheel immediately ──────────────────────────────────────
        double targetVelocity = Turret.INSTANCE.distanceToVelocity(
                startPose.getX(), startPose.getY(), Aliance.BLUE
        );
        Turret.INSTANCE.setVelocity(targetVelocity);
        Turret.INSTANCE.setHoodPosition(
                Turret.INSTANCE.distanceToPosition(startPose.getX(), startPose.getY(), Aliance.BLUE)
        );

        // ── Drive to shoot position ───────────────────────────────────────────
        followPath(p1);

        // ── Shoot first three ─────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake mode → drive to ball row 1 ────────────────────────────────
        intakeMode();
        followPath(p2);
        followPath(p3); // drive into balls
        followPath(p4); // return

        // ── Shoot second three ────────────────────────────────────────────────
        Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT);
        shootThree();

        // ── Intake mode → drive to ball row 2 ────────────────────────────────
        intakeMode();
        followPath(p5);
        followPath(p6); // drive into balls
        followPath(p7); // return

        // ── Shoot third three ─────────────────────────────────────────────────
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

        // Update velocity/hood for current position
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.BLUE));
        Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.BLUE));

        // Wait for flywheel (with timeout so auto doesn't hang)
        waitForVelocity();

        // Shoot position one
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_ONE);
        tickPeriodicFor(SPINDEXER_SETTLE_MS);
        flick(Spindexer.Position.POSITION_ONE);

        // Shoot position two
        Spindexer.INSTANCE.setToPosition(Spindexer.Position.POSITION_TWO);
        tickPeriodicFor(SPINDEXER_SETTLE_MS);
        flick(Spindexer.Position.POSITION_TWO);

        // Shoot position three
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

        // Keep turret aimed and velocity/hood updated
        if (Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT) {
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE);
            Turret.INSTANCE.setVelocity(Turret.INSTANCE.distanceToVelocity(x, y, Aliance.BLUE));
            Turret.INSTANCE.setHoodPosition(Turret.INSTANCE.distanceToPosition(x, y, Aliance.BLUE));
        } else {
            Turret.INSTANCE.setToAngle(270);
            // Keep flywheel warm during intake driving
            Turret.INSTANCE.setVelocity(500);
        }

        // Auto-rotate spindexer to free slot during intake
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
    // Paths — coordinates from Cheick's original BlueCloseSixBallAuto
    // ─────────────────────────────────────────────────────────────────────────
    private void buildPaths() {
        // p1: start → shoot position
        p1 = new Path(new BezierLine(
                new Pose(27.184, 130.041),
                new Pose(61.715, 94.224)
        ));
        p1.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        // p2: shoot position → above row 1
        p2 = new Path(new BezierLine(
                new Pose(61.715, 94.224),
                new Pose(61.560, 61.560)
        ));
        p2.setConstantHeadingInterpolation(Math.toRadians(180));

        // p3: drive into row 1 balls
        p3 = new Path(new BezierLine(
                new Pose(61.560, 59.672),
                new Pose(20.0, 59.904)
        ));
        p3.setConstantHeadingInterpolation(Math.toRadians(180));

        // p4: return from row 1
        p4 = new Path(new BezierLine(
                new Pose(20.0, 59.904),
                new Pose(61.728, 94.160)
        ));
        p4.setConstantHeadingInterpolation(Math.toRadians(180));

        // p5: shoot position → above row 2
        p5 = new Path(new BezierLine(
                new Pose(61.728, 94.160),
                new Pose(61.584, 83.480)
        ));
        p5.setConstantHeadingInterpolation(Math.toRadians(180));

        // p6: drive into row 2 balls
        p6 = new Path(new BezierLine(
                new Pose(61.584, 83.480),
                new Pose(20.0, 83.728)
        ));
        p6.setConstantHeadingInterpolation(Math.toRadians(180));

        // p7: return from row 2
        p7 = new Path(new BezierLine(
                new Pose(20.0, 83.536),
                new Pose(61.694, 94.048)
        ));
        p7.setConstantHeadingInterpolation(Math.toRadians(180));
    }
}