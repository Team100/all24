package org.team100.lib.fivebar;

import static java.lang.Math.sqrt;
import static java.lang.Math.pow;

/** A cartesian (x,y) point. */
public class Point {
    double x;
    double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distance(Point other) {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }

    public Point plus(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point minus(Point other) {
        return new Point(x - other.x, y - other.y);
    }

    public Point times(double scale) {
        return new Point(x * scale, y * scale);
    }

}
