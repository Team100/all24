package org.team100.lib.motion.mechanism;

import edu.wpi.first.math.MathUtil;

/**
 * Uses a motor and gears to produce rotational output, e.g. an arm joint.
 * 
 * Motor velocity and accel is higher than mechanism, required torque is lower,
 * using the supplied gear ratio.
 * 
 * The included encoder is the incremental motor encoder.
 */
public class TurretMechanism extends LimitedRotaryMechanism {
    public TurretMechanism(
            RotaryMechanism delegate,
            double minPositionRad,
            double maxPositionRad) {
        super(delegate, minPositionRad, maxPositionRad);
    }

    @Override
    public void setPosition(
            double outputPositionRad,
            double outputVelocityRad_S,
            double outputTorqueNm) {
        m_delegate.setPosition(
            MathUtil.inputModulus(outputPositionRad, m_minPositionRad, m_maxPositionRad),
            outputVelocityRad_S,
            outputTorqueNm);
    }
}
