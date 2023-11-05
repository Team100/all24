package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Various math utilities.
 */
public class MathUtil {
    private static final double EPSILON = 1e-6;

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
            return new ArrayList<Double>();
        }
    }

    public static boolean epsilonEquals(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }
}
