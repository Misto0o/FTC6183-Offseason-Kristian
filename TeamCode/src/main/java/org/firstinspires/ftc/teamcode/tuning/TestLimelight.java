package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import java.util.List;

@TeleOp(name = "TestLimelight", group = "Tuning")
public class TestLimelight extends OpMode {

    @Override
    public void init() {
        Drivetrain.getInstance().init(hardwareMap);
        Limelight.INSTANCE.initialize(hardwareMap);
    }

    @Override
    public void loop() {
        Drivetrain.getInstance().drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // ── High-level results ────────────────────────────────────────────────
        telemetry.addLine("── GOAL TAGS ────────────────────────────");
        telemetry.addData("Dist Blue (20)",  Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Dist Red  (24)",  Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));

        telemetry.addLine("── MOTIF ────────────────────────────────");
        int pattern = Limelight.INSTANCE.patternFromObelisk();
        telemetry.addData("Pattern", pattern == Limelight.GPP_PATTERN_ID ? "GPP" :
                pattern == Limelight.PGP_PATTERN_ID ? "PGP" :
                        pattern == Limelight.PPG_PATTERN_ID ? "PPG" : "NOT FOUND (" + pattern + ")");

        // ── Raw debug — shows EVERY fiducial the Limelight currently sees ─────
        // This is the key section: if the motif is visible but not matching,
        // you'll see the actual IDs here so we know what to fix.
        telemetry.addLine("── RAW FIDUCIALS (all seen) ─────────────");
        LLResult raw = Limelight.INSTANCE.getRawResult();
        if (raw == null) {
            telemetry.addData("LLResult", "NULL — limelight not connected");
        } else {
            List<LLResultTypes.FiducialResult> fiducials = raw.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                telemetry.addData("Fiducials", "NONE DETECTED");
            } else {
                telemetry.addData("Fiducial count", fiducials.size());
                for (int i = 0; i < fiducials.size(); i++) {
                    LLResultTypes.FiducialResult f = fiducials.get(i);
                    telemetry.addData("  Tag[" + i + "] ID", f.getFiducialId());
                    telemetry.addData("  Tag[" + i + "] X°", f.getTargetXDegrees());
                }
            }
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        Limelight.INSTANCE.stop();
    }
}