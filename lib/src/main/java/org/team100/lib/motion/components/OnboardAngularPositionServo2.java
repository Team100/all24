package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Because the 2025 angular encoder classes do not wind up, this is a version of
 * the position servo that understands that; it's almost a copy of
 * OnboardPositionServo.
 * 
 * TODO: replace the original with this, when it's done
 */
public class OnboardAngularPositionServo2 implements AngularPositionServo {

    private final Logger m_logger;
    /** Mechanism is measured in output shaft units, not motor units. */
    private final RotaryMechanism m_motor;
    /** Encoder measures the output 1:1. */
    private final RotaryPositionSensor m_encoder;
    private final double m_maxVel;
    private final PIDController m_controller;
    private final double m_period;
    private final Profile100 m_profile;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);
    // for calculating acceleration
    private double m_previousSetpoint = 0;
    private double m_prevTime;

    public OnboardAngularPositionServo2(
            Logger parent,
            RotaryMechanism motor,
            RotaryPositionSensor encoder,
            double maxVel,
            PIDController controller,
            Profile100 profile) {
        m_logger = parent.child(this);
        m_motor = motor;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_profile = profile;
    }

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    @Override
    public void reset() {
        m_controller.reset();
        m_prevTime = Timer.getFPGATimestamp();
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty())
            return;
        m_setpoint = new State100(position.getAsDouble(), velocity.getAsDouble());

        // TODO figure this out
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
    }

    @Override
    public void setPosition(double goalRad, double feedForwardTorqueNm) {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return;
        // measurement is output shaft rad
        double measurementRad = MathUtil.angleModulus(positionRad.getAsDouble());

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(MathUtil.angleModulus(goalRad - measurementRad) + measurementRad, 0.0);

        m_setpoint = new State100(
                MathUtil.angleModulus(m_setpoint.x() - measurementRad) + measurementRad,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        final double u_FB = m_controller.calculate(measurementRad, m_setpoint.x());
        final double u_FF = m_setpoint.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        // u_TOTAL is output shaft speed in rad/s
        final double u_TOTAL = MathUtil.clamp(u_FB + u_FF, -m_maxVel, m_maxVel);

        // pass the feedforward through unmodified
        m_motor.setVelocity(u_TOTAL, accel(u_TOTAL), feedForwardTorqueNm);
        m_logger.logDouble(Level.TRACE, "Desired velocity setpoint", () -> u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        m_logger.logState100(Level.TRACE, "Goal", () -> m_goal);
        m_logger.logDouble(Level.TRACE, "Measurement", () -> measurementRad);
        m_logger.logState100(Level.TRACE, "Setpoint", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "Controller Position Error", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error", m_controller::getVelocityError);
        m_logger.logDouble(Level.TRACE, "Feedforward Torque", () -> feedForwardTorqueNm);
    }

    /**
     * @return Current position measurement, radians.
     */
    @Override
    public OptionalDouble getPosition() {
        OptionalDouble position = m_encoder.getPositionRad();
        if (position.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(MathUtil.angleModulus(position.getAsDouble()));
    }

    /**
     * @return Current velocity, rad/s.
     */
    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRateRad_S();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        m_logger.logDouble(Level.TRACE, "Position Tolerance", m_controller::getPositionTolerance);
        m_logger.logDouble(Level.TRACE, "Velocity Tolerance", m_controller::getVelocityTolerance);
        m_logger.logBoolean(Level.TRACE, "At Setpoint", () -> atSetpoint);
        return atSetpoint;
    }

    @Override
    public boolean atGoal() {
        return atSetpoint()
                && MathUtil.isNear(
                        m_goal.x(),
                        m_setpoint.x(),
                        m_controller.getPositionTolerance())
                && MathUtil.isNear(
                        m_goal.v(),
                        m_setpoint.v(),
                        m_controller.getVelocityTolerance());
    }

    @Override
    public double getGoal() {
        return m_goal.x();
    }

    @Override
    public void stop() {
        m_motor.stop();
    }

    @Override
    public void close() {
        m_encoder.close();
    }

    /** for testing only */
    public State100 getSetpoint() {
        return m_setpoint;
    }

    ////////////////////////////////////////////////

    /**
     * there will be some jitter in dt, which will result in a small amount of
     * jitter in acceleration, and since this is a trailing difference there will be
     * a tiny bit of delay, compared to the actual profile. If this is
     * a problem, rewrite the profile class to expose the acceleration state and use
     * that instead.
     */
    private double accel(double setpoint) {
        double now = Timer.getFPGATimestamp();
        double dt = now - m_prevTime;
        m_prevTime = now;
        double accel = (setpoint - m_previousSetpoint) / dt;
        m_previousSetpoint = setpoint;
        return accel;
    }
}
