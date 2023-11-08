package org.team100.lib.motion.example1d;

public interface Kinematics1d {

    /**
     * Given a point in configuration space (e.g. joints),
     * returns a point in work space (e.g. cartesian) axis.
     */
    double forward(double x);

    /**
     * Given a point in work space (e.g. cartesian),
     * returns a point in configuration (e.g. joint) axis.
     */
    double inverse(double x);

}