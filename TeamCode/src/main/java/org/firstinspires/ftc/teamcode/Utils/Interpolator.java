package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 * 2D Interpolator
 * IMPORTANT: This class provides a 2D lookup table using Nearest Neighbor logic.
 * It is the core of the robot's "Auto-Aim" system, allowing it to determine
 * shooter settings based on any (X, Y) coordinate on the field.
 */
public class Interpolator {

    private final List<DataPoint> dataPoints = new ArrayList<>();

    private static class DataPoint {
        final double x, y, value;

        DataPoint(double x, double y, double value) {
            this.x     = x;
            this.y     = y;
            this.value = value;
        }

        /**
         * IMPORTANT: Standard Pythagorean distance formula for 2D space.
         */
        double distanceTo(double tx, double ty) {
            return Math.hypot(x - tx, y - ty);
        }
    }

    /**
     * IMPORTANT: Used to populate the table with calibrated field data.
     */
    public void addPoint(double x, double y, double value) {
        dataPoints.add(new DataPoint(x, y, value));
    }

    public double get(double x, double y) {
        if (dataPoints.isEmpty()) {
            throw new IllegalStateException("Interpolator has no data points");
        }

        // Sort by distance, take closest 4 points
        double totalWeight = 0;
        double weightedSum = 0;
        int k = Math.min(4, dataPoints.size());

        List<DataPoint> sorted = new ArrayList<>(dataPoints);
        sorted.sort((a, b) -> Double.compare(a.distanceTo(x, y), b.distanceTo(x, y)));

        for (int i = 0; i < k; i++) {
            DataPoint p = sorted.get(i);
            double dist = p.distanceTo(x, y);

            // If we're exactly on a point, return it immediately
            if (dist < 0.001) return p.value;

            double weight = 1.0 / (dist * dist); // inverse distance squared
            weightedSum += weight * p.value;
            totalWeight += weight;
        }

        return weightedSum / totalWeight;
    }
}
