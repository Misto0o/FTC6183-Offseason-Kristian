package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Turret;

import java.util.List;

@Config
public class Limelight {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}

    private Limelight3A limelight;

    // Camera mounting constants — measured from CAD and game manual. Used for just testing and a KeepSake
    public static double CAMERA_HEIGHT_IN = 17.5;
    public static double TAG_HEIGHT_IN    = 29.5;
    public static double CAMERA_TILT_DEG  = 12.07;

    public static double CAMERA_RADIUS_INCHES = 6.0;

    // ── AprilTag IDs ──────────────────────────────────────────────────────────
    public static final int BLUE_GOAL_ID   = 20;
    public static final int RED_GOAL_ID    = 24;
    public static final int GPP_PATTERN_ID = 21;
    public static final int PGP_PATTERN_ID = 22;
    public static final int PPG_PATTERN_ID = 23;

    // ── Pipeline indices ──────────────────────────────────────────────────────
    public static final int DETECTOR_PIPELINE = 9;
    public static final int FIDUCIAL_PIPELINE = 0;

    // ─────────────────────────────────────────────────────────────────────────
    public void initialize(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(FIDUCIAL_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    // NOTE: start/stop are kept for API compatibility, but the pipeline is never
    // fully killed — stopping the camera introduces a startup delay that causes
    // the turret's fine-tune to miss the first few frames after entering shoot mode.

    public void start() {
        if (limelight != null) limelight.start();
    }

    public void stop() {
        // Intentionally keep the camera running to avoid restart latency.
        // Results are simply ignored when not in shoot mode.
    }

    // ── Pipeline switching ────────────────────────────────────────────────────
    public void switchToDetector()  { if (limelight != null) limelight.pipelineSwitch(DETECTOR_PIPELINE); }
    public void switchToFiducial()  { if (limelight != null) limelight.pipelineSwitch(FIDUCIAL_PIPELINE); }

    public int getCurrentPipeline() {
        if (limelight == null) return -1;
        LLResult r = limelight.getLatestResult();
        return (r != null) ? r.getPipelineIndex() : -1;
    }

    public LLResult getRawResult() {
        if (limelight == null) return null;
        return limelight.getLatestResult();
    }

    // ── Fiducial helpers ──────────────────────────────────────────────────────

    /**
     * Returns true if the given tag ID is currently visible.
     * Use this instead of checking getTx() == 0, because tx can legitimately
     * be zero when the tag IS visible but perfectly centered.
     */
    public boolean hasTarget(int tagID) {
        if (limelight == null) return false;
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return false;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r == null || r.isEmpty()) return false;
        for (LLResultTypes.FiducialResult fiducial : r) {
            if (fiducial.getFiducialId() == tagID) return true;
        }
        return false;
    }

    /**
     * Returns estimated distance to the tag (inches).
     * Uses camera-to-tag pose from the LL, plus a fixed +16 in offset
     * on both axes to account for camera mounting position.
     * Returns 0 if tag is not visible.
     */
    public double distanceFromTag(int tagID) {
        if (limelight == null) return 0;
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return 0;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r == null || r.isEmpty()) return 0;
        for (LLResultTypes.FiducialResult fiducial : r) {
            if (fiducial.getFiducialId() == tagID) {
                double x = (fiducial.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 16;
                double z = (fiducial.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 16;
                return Math.sqrt(x * x + z * z);
            }
        }
        return 0;
    }

    public int patternFromObelisk() {
        if (limelight == null) return -1;
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return -1;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r == null || r.isEmpty()) return -1;
        for (LLResultTypes.FiducialResult fiducial : r) {
            int id = fiducial.getFiducialId();
            if (id == GPP_PATTERN_ID || id == PGP_PATTERN_ID || id == PPG_PATTERN_ID) return id;
        }
        return -1;
    }

    /**
     * Calculates the absolute field position of the robot center using
     * the camera's translation relative to a known field landmark (AprilTag Goal).
     */
    public double[] getPositionFromTargetTranslation(Aliance alliance) {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        int goalId = (alliance == Aliance.BLUE) ? BLUE_GOAL_ID : RED_GOAL_ID;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == goalId) {

                double camXOffsetInches = fiducial.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).x;
                double camZOffsetInches = fiducial.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).z;

                double targetFieldX = (alliance == Aliance.BLUE) ? Turret.BLUE_GOAL_X : Turret.RED_GOAL_X;
                double targetFieldY = (alliance == Aliance.BLUE) ? Turret.BLUE_GOAL_Y : Turret.RED_GOAL_Y;

                double globalCameraAngleRad = Math.toRadians(Pinpoint.INSTANCE.getHeading() + Turret.INSTANCE.getTurretAngle());

                double absoluteCamX = targetFieldX + (camZOffsetInches * Math.sin(globalCameraAngleRad)) + (camXOffsetInches * Math.cos(globalCameraAngleRad));
                double absoluteCamY = targetFieldY - (camZOffsetInches * Math.cos(globalCameraAngleRad)) + (camXOffsetInches * Math.sin(globalCameraAngleRad));

                double cameraRadiusInches = 6.0;
                double robotFieldX = absoluteCamX - (cameraRadiusInches * Math.cos(globalCameraAngleRad));
                double robotFieldY = absoluteCamY - (cameraRadiusInches * Math.sin(globalCameraAngleRad));

                return new double[]{ robotFieldX, robotFieldY };
            }
        }
        return null;
    }

    // ── MT1 -> field-corner-origin (Pinpoint-style) conversion ──────────────
    // This is your friend's CONFIRMED WORKING transform (from their LimeLight.java,
    // translateLimelightPoseToPedro), ported from Pedro-Pose-returning form into
    // plain {x, y, headingDeg} doubles so it drops straight into
    // Pinpoint.relocalizePositionFromTag(x, y) / Pinpoint.updatePosition(...).
    // Steps, in order, exactly as their code does them:
    //   1. swap X and Y
    //   2. add 72 to X
    //   3. negate Y
    //   4. add 72 to Y
    //   5. heading: normalize to [0,360), then subtract 90
    // Do NOT reorder these steps — swap-then-shift is not the same transform as
    // shift-then-swap, and this exact order is what's confirmed working on their
    // (very similar) robot.
    public double[] getMT1Pose() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return null;

        Position posIn = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double rawX = posIn.x;
        double rawY = posIn.y;

        if (Math.abs(rawX) < 0.01 && Math.abs(rawY) < 0.01) return null;

        // Step 1: swap
        double swappedX = rawY;
        double swappedY = rawX;

        // Step 2 & 3 & 4
        double fieldX = swappedX + 72.0;
        double fieldY = (-swappedY) + 72.0;

        if (fieldX < 0 || fieldX > 144 || fieldY < 0 || fieldY > 144) return null;

        return new double[]{ fieldX, fieldY };
    }

    /**
     * MT1 heading, using the same normalize-then-minus-90 convention as the
     * confirmed-working friend transform. Degrees.
     */
    public double getMT1Yaw() {
        if (limelight == null) return -1;
        LLResult result = limelight.getLatestResult();
        if (result == null) return -1;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return -1;

        double yaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        if (yaw < 0) yaw += 360;
        yaw -= 90;
        return yaw;
    }

    /**
     * MT1 raw pose in inches, NO conversion applied — straight from the LL.
     * Use this to sanity-check getMT1Pose()'s transform against known field spots.
     */
    public double[] getMT1PoseRaw() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        Position posIn = botpose.getPosition().toUnit(DistanceUnit.INCH);
        return new double[]{ posIn.x, posIn.y };
    }

    // ── MT2 (kept for reference / rollback — not currently used) ─────────────

    public boolean hasMegaTagPose() {
        if (limelight == null) return false;
        LLResult result = limelight.getLatestResult();
        if (result == null) return false;
        Pose3D botpose = result.getBotpose_MT2();
        return botpose != null;
    }

    /**
     * Converts the current MT2 botpose into Pinpoint field coordinates (inches).
     * IMPORTANT: This does NOT push robot orientation into the Limelight itself.
     * MT2 needs fresh yaw fed in via setRobotOrientation(...) BEFORE you call this.
     */
    public double[] getMegaTagPose() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return null;
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return null;

        Position posIn = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double rawX = posIn.x;
        double rawY = posIn.y;

        if (Math.abs(rawX) < 0.01 && Math.abs(rawY) < 0.01) return null;

        double swappedX = rawY;
        double swappedY = rawX;
        double fieldX = swappedX + 72.0;
        double fieldY = (-swappedY) + 72.0;

        if (fieldX < 0 || fieldX > 144 || fieldY < 0 || fieldY > 144) return null;

        return new double[]{ fieldX, fieldY };
    }

    public double getMegaTagYaw() {
        if (limelight == null) return -1;
        LLResult result = limelight.getLatestResult();
        if (result == null) return -1;

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return -1;

        double yaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        return ((yaw % 360) + 360) % 360;
    }

    public double[] getMegaTagPoseRaw() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null) return null;
        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return null;

        Position posIn = botpose.getPosition().toUnit(DistanceUnit.INCH);
        return new double[]{ posIn.x, posIn.y };
    }

    // ── Snapshot helpers used by Teleop / TestLimelight ──────────────────────

    public double[] getSnapshotPose() {
        return getMT1Pose();
    }

    public double[] getAveragedSnapshotPose(int samples) {
        double sumX = 0, sumY = 0;
        int valid = 0;
        for (int i = 0; i < samples; i++) {
            double[] pose = getSnapshotPose();
            if (pose != null && pose[0] > 1 && pose[0] < 143 && pose[1] > 1 && pose[1] < 143) {
                sumX += pose[0];
                sumY += pose[1];
                valid++;
            }
            try { Thread.sleep(20); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }
        if (valid == 0) return null;
        return new double[]{ sumX / valid, sumY / valid };
    }

    public void setRobotOrientation(double yawDegrees) {
        if (limelight != null) {
            limelight.updateRobotOrientation(yawDegrees);
        }
    }
}