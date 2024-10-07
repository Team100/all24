package org.team100.lib.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Timing constraints govern the assignment of a schedule to a path, creating a
 * trajectory. Different implementations focus on different aspects, e.g.
 * tippiness, wheel slip, etc. Different maneuvers may want different
 * constraints, e.g. some should be slow and precise, others fast and risky.
 */
public interface TimingConstraint {
    /**
     * Maximum allowed velocity m/s.
     */
    NonNegativeDouble getMaxVelocity(Pose2dWithMotion state);

    class NonNegativeDouble {
        private final double m_value;

        public NonNegativeDouble(double value) {
            if (value < 0)
                throw new IllegalArgumentException();
            m_value = value;
        }

        public double getValue() {
            return m_value;
        }
    }

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

        /**
         * @param minAccel negative
         * @param maxAccel positive
         */
        public MinMaxAcceleration(double minAccel, double maxAccel) {
            if (minAccel > 0) {
                throw new IllegalArgumentException();
            }
            if (maxAccel < 0) {
                throw new IllegalArgumentException();
            }
            m_minAccel = minAccel;
            m_maxAccel = maxAccel;
        }

        /** Always negative (or zero). */
        public double getMinAccel() {
            return m_minAccel;
        }

        /** Always positive (or zero). */
        public double getMaxAccel() {
            return m_maxAccel;
        }
    }
}
