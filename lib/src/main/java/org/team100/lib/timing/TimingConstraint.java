package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

public interface TimingConstraint {
    /**
     * Maximum allowed velocity m/s.
     */
    double getMaxVelocity(Pose2dWithMotion state);

    /** 
     * Minimum and maximum allowed acceleration m/s^2.
     */
    MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S);

    class MinMaxAcceleration {
        public static final MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        private final double m_minAccel;
        private final double m_maxAccel;

        private MinMaxAcceleration() {
            this(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        public MinMaxAcceleration(double minAccel, double maxAccel) {
            if (getMinAccel() > getMaxAccel()) {
                throw new IllegalArgumentException();
            }
            m_minAccel = minAccel;
            m_maxAccel = maxAccel;
        }

        public double getMinAccel() {
            return m_minAccel;
        }

        public double getMaxAccel() {
            return m_maxAccel;
        }
    }
}
