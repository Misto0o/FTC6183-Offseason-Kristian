package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Limelight {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}

    private Limelight3A limelight;

    public static final int BLUE_GOAL_ID   = 20;
    public static final int RED_GOAL_ID    = 24;
    public static final int GPP_PATTERN_ID = 21;
    public static final int PGP_PATTERN_ID = 22;
    public static final int PPG_PATTERN_ID = 23;

    public void initialize(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
        } catch (Exception e) {
            limelight = null; // unplugged or not configured — all methods null-check safely
        }
    }

    public void start() { limelight.start(); }
    public void stop()  { limelight.stop();  }

    public double angleFromTag(double tagID) {
        if (limelight == null) return -1;
        limelight.start();
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return -1;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r.isEmpty()) return -1;
        for (LLResultTypes.FiducialResult fiducial : r) {
            if (fiducial.getFiducialId() == tagID) {
                return fiducial.getTargetXDegrees();
            }
        }
        return -1;
    }

    public double distanceFromTag(double tagID) {
        if (limelight == null) return 0;
        limelight.start();
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return 0;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r.isEmpty()) return 0;
        for (LLResultTypes.FiducialResult fiducial : r) {
            if (fiducial.getFiducialId() == tagID) {
                double x = (fiducial.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch) + 16;
                double z = (fiducial.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch) + 16;
                return Math.sqrt(x * x + z * z); // plain Pythagorean, no Pedro Vector needed
            }
        }
        return 0;
    }

    public int patternFromObelisk() {
        if (limelight == null) return -1;
        limelight.start();
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return -1;
        List<LLResultTypes.FiducialResult> r = latest.getFiducialResults();
        if (r.isEmpty()) return -1;
        for (LLResultTypes.FiducialResult fiducial : r) {
            int id = fiducial.getFiducialId();
            if (id == GPP_PATTERN_ID || id == PGP_PATTERN_ID || id == PPG_PATTERN_ID) {
                return id;
            }
        }
        return -1;
    }
}