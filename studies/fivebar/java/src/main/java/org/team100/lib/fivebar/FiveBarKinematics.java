package org.team100.lib.fivebar;

import static java.lang.Math.sqrt;
import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.PI;

/**
 * Kinematics of 2-dof 5-bar planar linkage with one grounded bar.
 * 
 * Adapted from http://charm.stanford.edu/ME327/JaredAndSam
 * 
 * Which is itself adapted from "The Pantograph Mk-II: A Haptic Instrument"
 * Hayward, 2005
 * 
 * See pantograph.png for the coordinates used here.
 */
public class FiveBarKinematics {

    /**
     * Computes inverse kinematics.
     * 
     * @param scenario simulation geometry
     * @param x3,y3    position of end effector ("P3" in the diagram), meters
     * @return the angles of the proximal links.
     */
    public static ActuatorAngles inverse(Scenario scenario, double x3, double y3) {
        double P13 = sqrt((pow(x3, 2)) + (pow(y3, 2)));
        double P53 = sqrt((pow((x3 + scenario.a5), 2)) + (pow(y3, 2)));

        double alphaOne = acos(
                ((pow(scenario.a1, 2)) + (pow(P13, 2)) - (pow(scenario.a2, 2))) / (2 * scenario.a1 * P13));
        double betaOne = atan2(y3, -x3);
        double thetaOne = PI - alphaOne - betaOne;

        double alphaFive = atan2(y3, x3 + scenario.a5);
        double betaFive = acos(
                ((pow(P53, 2)) + (pow(scenario.a4, 2)) - (pow(scenario.a3, 2))) / (2 * P53 * scenario.a4));
        double thetaFive = alphaFive + betaFive;

        return new ActuatorAngles(thetaOne, thetaFive);
    }

    /**
     * Computes forward kinematics.
     * 
     * @param scenario simulation geometry
     * @param t1       the angle between a1 and a5
     * @param t5       the angle between a4 and a5
     * @returns the five joint positions, and also the center of the hypotenuse
     */
    public static JointPositions forward(Scenario scenario, double t1, double t5) {
        // by definition
        double x1 = scenario.x1;
        double y1 = scenario.y1;
        Point P1 = new Point(x1, y1);

        double x2 = scenario.a1 * cos(t1);
        double y2 = scenario.a1 * sin(t1);
        Point P2 = new Point(x2, y2);

        double x4 = scenario.a4 * cos(t5) - scenario.a5;
        double y4 = scenario.a4 * sin(t5);
        Point P4 = new Point(x4, y4);

        double P2Ph = (pow(scenario.a2, 2) - pow(scenario.a3, 2) + pow(P4.distance(P2), 2)) / (2 * P4.distance(P2));
        Point Ph = P2.plus((P4.minus(P2)).times((P2Ph / P2.distance(P4))));
        double P3Ph = sqrt(pow(scenario.a2, 2) - pow(P2Ph, 2));

        double x3 = Ph.x + (P3Ph / P2.distance(P4)) * (y4 - y2);
        double y3 = Ph.y - (P3Ph / P2.distance(P4)) * (x4 - x2);

        Point P3 = new Point(x3, y3);

        double x5 = scenario.x1 - scenario.a5;
        double y5 = scenario.y1;
        Point P5 = new Point(x5, y5);

        return new JointPositions(P1, P2, P3, P4, P5, Ph);
    }

    private FiveBarKinematics() {
        //
    }

}