package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 * 2D lookup table with bilinear interpolation.
 * Falls back to nearest neighbor if the query point is outside the grid
 * or if the grid doesn't have all 4 surrounding corners. (thanks cheick)
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

        // Exact match
        for (DataPoint p : dataPoints) {
            if (Math.abs(p.x - x) < 1e-9 && Math.abs(p.y - y) < 1e-9) {
                return p.value;
            }
        }

        // Bilinear, then nearest neighbor fallback
        Double bilinear = tryBilinear(x, y);
        return bilinear != null ? bilinear : nearestNeighbor(x, y);
    }

    private Double tryBilinear(double x, double y) {
        // Step 1: Find tightest bounding x and y values
        double lowerX = Double.NEGATIVE_INFINITY;
        double upperX = Double.POSITIVE_INFINITY;
        double lowerY = Double.NEGATIVE_INFINITY;
        double upperY = Double.POSITIVE_INFINITY;

        for (DataPoint p : dataPoints) {
            if (p.x <= x && p.x > lowerX) lowerX = p.x;
            if (p.x >= x && p.x < upperX) upperX = p.x;
            if (p.y <= y && p.y > lowerY) lowerY = p.y;
            if (p.y >= y && p.y < upperY) upperY = p.y;
        }

        if (Double.isInfinite(lowerX) || Double.isInfinite(upperX) ||
                Double.isInfinite(lowerY) || Double.isInfinite(upperY)) {
            return null;
        }

        // Step 2: Look up all 4 corners
        DataPoint bl = findPoint(lowerX, lowerY); // bottom-left
        DataPoint br = findPoint(upperX, lowerY); // bottom-right
        DataPoint tl = findPoint(lowerX, upperY); // top-left
        DataPoint tr = findPoint(upperX, upperY); // top-right

        if (bl == null || br == null || tl == null || tr == null) {
            return null;
        }

        // Step 3: Bilinear interpolation
        double tx = (upperX - lowerX) < 1e-9 ? 0.0 : (x - lowerX) / (upperX - lowerX);
        double ty = (upperY - lowerY) < 1e-9 ? 0.0 : (y - lowerY) / (upperY - lowerY);

        return lerp(ty, lerp(tx, bl.value, br.value),
                lerp(tx, tl.value, tr.value));
    }

    private DataPoint findPoint(double x, double y) {
        for (DataPoint p : dataPoints) {
            if (Math.abs(p.x - x) < 1e-9 && Math.abs(p.y - y) < 1e-9) {
                return p;
            }
        }
        return null;
    }

    private double nearestNeighbor(double x, double y) {
        DataPoint nearest = dataPoints.get(0);
        double minDist = nearest.distanceTo(x, y);

        for (DataPoint p : dataPoints) {
            double dist = p.distanceTo(x, y);
            if (dist < minDist) {
                minDist = dist;
                nearest = p;
            }
        }

        return nearest.value;
    }

    private double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }
}