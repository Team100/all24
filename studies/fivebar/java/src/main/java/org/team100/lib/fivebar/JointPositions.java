package org.team100.lib.fivebar;

/**
 * The x and y positions of all the joints and the center of the hypotenuse
 */
public class JointPositions {
    final Point P1;
    final Point P2;
    final Point P3;
    final Point P4;
    final Point P5;
    final Point Ph;

    public JointPositions(Point p1, Point p2, Point p3, Point p4, Point p5, Point ph) {
        P1 = p1;
        P2 = p2;
        P3 = p3;
        P4 = p4;
        P5 = p5;
        Ph = ph;
    }

}
