package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.profile.Profile100;
import org.team100.lib.util.Util;

/**
 * Wraps an angular position servo, supplying it with the correct feed forward
 * torque for gravity compensation.
 */
public class GravityServo2 implements GravityServoInterface {
    private final AngularPositionServo m_servo;
    /** Max gravity torque, newton-meters */
    private final double m_gravityNm; // = 5.0;
    /** Offset from horizontal */
    private final double m_offsetRad; // = 0.0;

    public GravityServo2(
            AngularPositionServo servo,
            double gravityNm,
            double offsetRad) {
        m_servo = servo;
        m_gravityNm = gravityNm;
        m_offsetRad = offsetRad;
    }

    @Override
    public void reset() {
        m_servo.reset();
    }

    @Override
    public OptionalDouble getPositionRad() {
        return m_servo.getPosition();
    }

    @Override
    public void setState(State100 goal) {
        OptionalDouble optPos = getPositionRad();
        if (optPos.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        final double gravityTorqueNm = m_gravityNm * Math.cos(mechanismPositionRad + m_offsetRad);
        m_servo.setPositionWithVelocity(goal.x(), goal.v(), gravityTorqueNm);
    }

    @Override
    public void stop() {
        m_servo.stop();
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_servo.setProfile(profile);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_servo.setTorqueLimit(torqueNm);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

}
