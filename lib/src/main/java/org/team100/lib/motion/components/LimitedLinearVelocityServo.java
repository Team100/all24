package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Wraps a linear velocity servo and smooths and limits its setpoint.
 */
public class LimitedLinearVelocityServo implements LinearVelocityServo {

    private final LinearVelocityServo m_servo;
    private final double m_maxVel;
    /** The limiter keeps state. */
    private final SlewRateLimiter m_limiter;

    /**
     * 
     * @param servo    the servo to wrap
     * @param maxVel   maximum velocity
     * @param maxAccel maximum acceleration
     * @param maxDecel maximum deceleration: usually mechanisms can slow down faster
     *                 than they can speed up. this should be a negative number.
     */
    public LimitedLinearVelocityServo(
            LinearVelocityServo servo,
            double maxVel,
            double maxAccel,
            double maxDecel) {
        if (maxDecel > 0)
            throw new IllegalArgumentException("max decel must be non-positive");
        m_servo = servo;
        m_maxVel = maxVel;
        m_limiter = new SlewRateLimiter(maxAccel, maxDecel, 0);
        reset();
    }

    @Override
    public void reset() {
        m_servo.reset();
        m_limiter.reset(0);
    }

    @Override
    public void setVelocity(double goal) {
        double setpoint = m_limiter.calculate(goal);
        setpoint = MathUtil.clamp(setpoint, -m_maxVel, m_maxVel);
        m_servo.setVelocity(setpoint);
    }

    @Override
    public void setVelocity(double goal, double goal_2) {
        double setpoint = m_limiter.calculate(goal);
        setpoint = MathUtil.clamp(setpoint, -m_maxVel, m_maxVel);
        m_servo.setVelocity(setpoint);
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_servo.getVelocity();
    }

    @Override
    public void stop() {
        m_servo.stop();
        m_limiter.reset(0);
    }

    @Override
    public OptionalDouble getDistance() {
        return m_servo.getDistance();
    }

    @Override
    public double getSetpoint() {
        return m_servo.getSetpoint();
    }
}
