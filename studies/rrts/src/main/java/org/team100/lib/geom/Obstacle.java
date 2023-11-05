package org.team100.lib.geom;

import java.awt.Color;
import java.awt.Shape;


public interface Obstacle {
    Shape shape();
    Color color();

    /**
     * Returns the distance from the exterior wall of the obstacle to a point.
     * A return of 0 means the point is on the circumference, a negative
     * value means the point is on the interior of the obstacle.
     *
     * @param x coordinate of the point to test
     * @param y coordinate of the point to test
     * @return distance from exterior wall of obstacle
     */
    double distToPoint(double x, double y);

    /**
     * Returns the distance from the exterior wall of the obstacle to a line
     * segment.  Any negative value indicates that the line intersects the
     * obstacle.
     *
     * @param x1 x-coordinate of the first end-point of the segment to test
     * @param y1 y-coordinate of the first end-point of the segment to test
     * @param x2 x-coordinate of the second end-point of the segment to test
     * @param y2 y-coordinate of the second end-point of the segment to test
     * @return distance from the exterior wall of obstacle
     */
    double distToSeg(double x1, double y1, double x2, double y2);
}
