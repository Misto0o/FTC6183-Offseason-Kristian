package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import java.util.List;

@TeleOp(name = "TestLimelight", group = "Tuning")
public class TestLimelight extends OpMode {

    private Aliance alliance = Aliance.BLUE;
    private boolean lastDU = false;
    private boolean lastDD = false;
    private boolean initializedPose = false;

    @Override
    public void init() {
        Drivetrain.getInstance().init(hardwareMap);
        Turret.INSTANCE.initialize(hardwareMap);
        Pinpoint.INSTANCE.init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);

        initializedPose = false;
        telemetry.addLine("Use DPad UP (Blue) or DPad DOWN (Red) to test alliances");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Handle alliance selection toggles during initialization
        boolean du = gamepad1.dpad_up;
        boolean dd = gamepad1.dpad_down;
        if (du && !lastDU) alliance = Aliance.BLUE;
        if (dd && !lastDD) alliance = Aliance.RED;
        lastDU = du; lastDD = dd;

        // 1. Check if the Limelight sees a tag at all
        LLResult result = Limelight.INSTANCE.getRawResult();
        boolean seesTag = (result != null && result.isValid() &&
                result.getFiducialResults() != null &&
                !result.getFiducialResults().isEmpty());

        if (seesTag) {
            // CHANGED: switched from MT2 (getMegaTagYaw + setRobotOrientation +
            // getMegaTagPose) to MT1 (getMT1Yaw + getMT1Pose). MT1 solves both
            // rotation and translation fresh from the tag every frame, so it does
            // NOT need setRobotOrientation() fed in first — no more chicken-and-egg
            // startup seeding needed.
            double startupYaw = Limelight.INSTANCE.getMT1Yaw();
            double[] startupPose = Limelight.INSTANCE.getMT1Pose();

            if (startupPose != null) {
                // Seed Pinpoint with both the accurate pose and heading
                Pinpoint.INSTANCE.updatePosition(new Pose2D(
                        DistanceUnit.INCH, startupPose[0], startupPose[1], AngleUnit.DEGREES, startupYaw
                ));
                initializedPose = true;
                telemetry.addData("Status", "Tag found! Seeding Pinpoint dynamically.");
                telemetry.addData("Seeded Pos", String.format("(%.1f, %.1f) @ %.1f°", startupPose[0], startupPose[1], startupYaw));
            } else {
                telemetry.addLine("Tag seen, but position math out-of-bounds. Adjusting...");
            }
        } else {
            // Fallback: No tag visible yet, set positioning anchor to the requested corner reference
            double fallbackX = (alliance == Aliance.BLUE) ? 135.5 : 8.5;
            double fallbackY = 9.0;
            double fallbackHeading = (alliance == Aliance.BLUE) ? 0.0 : 180.0;

            Pinpoint.INSTANCE.updatePosition(new Pose2D(
                    DistanceUnit.INCH, fallbackX, fallbackY, AngleUnit.DEGREES, fallbackHeading
            ));
            telemetry.addData("Status", "No tag visible. Seeding to localization corner reference.");
            telemetry.addData("Corner Ref", String.format("(%.1f, %.1f) @ %.1f°", fallbackX, fallbackY, fallbackHeading));
        }

        telemetry.addData("Selected Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        Pinpoint.INSTANCE.periodic();

        Turret.INSTANCE.setToAngle(Turret.TURRET_PARKED_ANGLE);
        Turret.INSTANCE.periodic();

        // NOTE: setRobotOrientation() is no longer needed for MT1 (it solves
        // rotation itself every frame), but it's left here harmless in case you
        // ever fall back to MT2 — costs nothing to keep feeding it.
        Limelight.INSTANCE.setRobotOrientation(Pinpoint.INSTANCE.getHeading());

        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        telemetry.addLine("── GOAL TAGS ────────────────────────────");
        telemetry.addData("Dist Blue (20)",  Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Dist Red  (24)",  Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));

        // CHANGED: MT2 -> MT1 throughout. This is your main calibration-check
        // screen: compare "MT1 raw inches" + "MT1 converted" against where the
        // robot actually is at a few different known field spots.
        telemetry.addLine("── MT1 DATA ─────────────────────────────");
        double[] raw = Limelight.INSTANCE.getMT1PoseRaw();
        double[] converted = Limelight.INSTANCE.getMT1Pose();
        double yaw = Limelight.INSTANCE.getMT1Yaw();

        if (raw != null) {
            telemetry.addData("MT1 raw inches", String.format("(%.1f, %.1f)", raw[0], raw[1]));
        } else {
            telemetry.addData("MT1 raw inches", "NO TAG");
        }
        if (converted != null) {
            telemetry.addData("MT1 converted", String.format("(%.1f, %.1f)", converted[0], converted[1]));
        } else {
            telemetry.addData("MT1 converted", "NO TAG / OUT OF BOUNDS");
        }
        telemetry.addData("MT1 yaw", yaw);

        // ── PINPOINT SYSTEM VERIFICATION ─────────────────────────────────────
        telemetry.addLine("── PINPOINT ODOM ────────────────────────");
        telemetry.addData("Odo Heading", String.format("%.1f°", Pinpoint.INSTANCE.getHeading()));
        telemetry.addData("Odo X / Y", String.format("(%.1f, %.1f)", Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY()));

        telemetry.update();
    }

    @Override
    public void stop() {
        Limelight.INSTANCE.stop();
    }
}