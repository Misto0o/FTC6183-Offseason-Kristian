package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 * 2D lookup table with inverse distance weighting (IDW) interpolation.
 * Works on scattered, irregular data points — no grid required.
 */
public class Interpolator {

    private final List<DataPoint> dataPoints = new ArrayList<>();

    private static class DataPoint {
        final double x, y, value;

        DataPoint(double x, double y, double value) {
            this.x = x;
            this.y = y;
            this.value = value;
        }

        double distanceTo(double tx, double ty) {
            return Math.hypot(x - tx, y - ty);
        }
    }

    public void addPoint(double x, double y, double value) {
        dataPoints.add(new DataPoint(x, y, value));
    }

    public int size() {
        return dataPoints.size();
    }

    public double get(double x, double y) {
        if (dataPoints.isEmpty()) {
            throw new IllegalStateException("Interpolator has no data points");
        }

        double weightedSum = 0, totalWeight = 0;
        for (DataPoint p : dataPoints) {
            double dist = p.distanceTo(x, y);
            if (dist < 1e-9) return p.value; // exact match
            double w = 1.0 / (dist * dist);  // inverse square distance
            weightedSum += w * p.value;
            totalWeight += w;
        }
        return weightedSum / totalWeight;
    }
}