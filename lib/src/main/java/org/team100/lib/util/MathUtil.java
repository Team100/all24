package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Various math utilities.
 */
public class MathUtil {
    public static final double EPSILON = 1e-6;

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        double disc = b * b - 4 * a * c;

        if (epsilonEquals(disc, 0.0)) {
            return List.of(-b / (2 * a));
        } else if (disc > 0.0) {
            return List.of(
                    (-b + Math.sqrt(disc)) / (2 * a),
                    (-b - Math.sqrt(disc)) / (2 * a));
        } else {
            return new ArrayList<>();
        }
    }

    public static boolean epsilonEquals(double x, double y) {
        return epsilonEquals(x, y, EPSILON);
    }

    public static boolean epsilonEquals(double x, double y, double epsilon) {
        return Math.abs(x - y) < epsilon;
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    private MathUtil() {
    }
}
