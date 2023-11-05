package org.team100.lib.math;

import static java.lang.Math.sqrt;

import java.util.List;

public class Util {

    /**
     * returns roots of the quadratic
     * see KrisLibrary/planning/ParabolicRamp.cpp
     */
    public static List<Double> quadratic(double a, double b, double c) {
        if (a == 0) {
            if (b == 0) {
                if (c == 0)
                    return List.of();
                return List.of();
            }
            return List.of(-c / b);
    
        }
        if (c == 0) { // det = b^2
            if (b == 0) // just y=ax^2
                return List.of(0.0);
            return List.of(0.0, -b / a);
        }
        double det = b * b - 4.0 * a * c;
        if (det < 0.0)
            return List.of();
        if (det == 0.0) {
            return List.of(-b / (2.0 * a));
        }
        det = sqrt(det);
        double x1;
        double x2;
        if (Math.abs(-b - det) < Math.abs(a))
            x1 = 0.5 * (-b + det) / a;
        else
            x1 = 2.0 * c / (-b - det);
        if (Math.abs(-b + det) < Math.abs(a))
            x2 = 0.5 * (-b - det) / a;
        else
            x2 = 2.0 * c / (-b + det);
        return List.of(x1, x2);
    }
    
}
