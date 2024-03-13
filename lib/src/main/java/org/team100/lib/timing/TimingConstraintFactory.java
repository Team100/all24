package org.team100.lib.timing;

import java.util.List;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

public class TimingConstraintFactory {
    private final SwerveKinodynamics m_limits;

    public TimingConstraintFactory(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    /**
     * Return wheel speed, yaw rate, and centripetal acceleration constraints with
     * reasonable scale to prevent too-fast spinning and to slow down in sharp
     * curves.
     * 
     * If you want to adjust these scales, make a new factory method while you
     * figure out what you need, don't just change them here.
     */
    public List<TimingConstraint> allGood() {
        return scaled(0.4, 0.2);
    }

    /**
     * Use absolute max as the constraints.  Shouldn't be used on a real robot.
     */
    public List<TimingConstraint> forTest() {
        return scaled(1.0, 1.0);
    }

    private List<TimingConstraint> scaled(double centripetalScale, double yawRateScale) {
        return List.of(
                new SwerveDriveDynamicsConstraint(m_limits),
                new YawRateConstraint(m_limits, yawRateScale),
                new CentripetalAccelerationConstraint(m_limits, centripetalScale));
    }

}
