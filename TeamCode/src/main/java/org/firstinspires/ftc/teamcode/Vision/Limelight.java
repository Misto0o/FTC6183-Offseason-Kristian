package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.robot.Pinpoint;
import org.firstinspires.ftc.teamcode.robot.Turret;


import java.util.List;
@Config
public class Limelight {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}

    private Limelight3A limelight;

    // Camera mounting constants — measured from CAD and game manual Used for just testing and a KeepSake
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

        // Use the specific goal ID your robot is tracking
        int goalId = (alliance == Aliance.BLUE) ? BLUE_GOAL_ID : RED_GOAL_ID;

        // Find the targeted fiducial in our results array
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == goalId) {

                // 1. Get the camera's position relative to the center face of the AprilTag
                // Target Space: Z is distance away from tag face, X is horizontal offset from tag center
                double camXOffsetInches = fiducial.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).x;
                double camZOffsetInches = fiducial.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).z;

                // 2. Identify the absolute field coordinates of your target goals (using your corner map 0-144)
                double targetFieldX = (alliance == Aliance.BLUE) ? Turret.BLUE_GOAL_X : Turret.RED_GOAL_X;
                double targetFieldY = (alliance == Aliance.BLUE) ? Turret.BLUE_GOAL_Y : Turret.RED_GOAL_Y;

                // 3. Since the camera is on a rotating turret, we calculate the global heading of the camera body
                double globalCameraAngleRad = Math.toRadians(Pinpoint.INSTANCE.getHeading() + Turret.INSTANCE.getTurretAngle());

                // 4. Translate camera coordinate vectors back into global field coordinates
                double absoluteCamX = targetFieldX + (camZOffsetInches * Math.sin(globalCameraAngleRad)) + (camXOffsetInches * Math.cos(globalCameraAngleRad));
                double absoluteCamY = targetFieldY - (camZOffsetInches * Math.cos(globalCameraAngleRad)) + (camXOffsetInches * Math.sin(globalCameraAngleRad));

                // 5. Account for the physical distance from the camera lens to the center of your drivetrain
                double cameraRadiusInches = 6.0; // Change this to your robot's actual measurement
                double robotFieldX = absoluteCamX - (cameraRadiusInches * Math.cos(globalCameraAngleRad));
                double robotFieldY = absoluteCamY - (cameraRadiusInches * Math.sin(globalCameraAngleRad));

                return new double[]{ robotFieldX, robotFieldY };
            }
        }
        return null;
    }

    public boolean hasMegaTagPose() {
        if (limelight == null) return false;
        LLResult result = limelight.getLatestResult();
        if (result == null) return false;
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D botpose =
                result.getBotpose_MT2();
        return botpose != null;
    }

    public double[] getMegaTagPose() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        // MegaTag2 requires the IMU orientation to be fed into the device
        // immediately prior to querying this layout loop.
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D botpose =
                result.getBotpose_MT2();
        if (botpose == null) return null;

        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return null;

        // Convert raw meters to inches
        double rawXInches = botpose.getPosition().x * 39.3701;
        double rawYInches = botpose.getPosition().y * 39.3701;

        if (Math.abs(rawXInches) < 0.01 && Math.abs(rawYInches) < 0.01) return null;

        double pinpointX = rawXInches * 2.295 + 46.0;
        double pinpointY = rawYInches * 0.745 + 159.0;

        if (pinpointX < 0 || pinpointX > 144 || pinpointY < 0 || pinpointY > 144) return null;

        return new double[]{ pinpointX, pinpointY };
    }


    public double[] getMegaTagPoseRaw() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D botpose =
                result.getBotpose_MT2();
        if (botpose == null) return null;

        double x = botpose.getPosition().x * 39.3701;
        double y = botpose.getPosition().y * 39.3701;

        return new double[]{ x, y };
    }

    public double[] getSnapshotPose() {
        return getMegaTagPose(); // getAveragedSnapshotPose() already calls this
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