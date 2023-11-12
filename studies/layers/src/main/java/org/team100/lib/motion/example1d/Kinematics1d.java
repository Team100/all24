package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.Configuration;
import org.team100.lib.motion.example1d.framework.Kinematics;
import org.team100.lib.motion.example1d.framework.Workstate;

public interface Kinematics1d extends Kinematics<Workstate<Double>, Configuration<Double>> {

    /**
     * Given a point in configuration space (e.g. joints),
     * returns a point in work space (e.g. cartesian) axis.
     */
    @Override
    Workstate<Double> forward(Configuration<Double> x);

    /**
     * Given a point in work space (e.g. cartesian),
     * returns a point in configuration (e.g. joint) axis.
     */
    @Override
    Configuration<Double> inverse(Workstate<Double> x);

}