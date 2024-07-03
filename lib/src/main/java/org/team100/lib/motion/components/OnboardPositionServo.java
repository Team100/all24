package org.team100.lib.motion.components;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.VelocityMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Positional control via onboard control of motor velocity.
 */
public class OnboardPositionServo<T extends Measure100> implements PositionServo<T> {
    private final Telemetry.Logger t;
    private final VelocityMotor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final double m_maxVel;
    private final PIDController m_controller;
    private final double m_period;
    private final String m_name;
    private final Profile100 m_profile;
    private final T m_instance;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);
    // for calculating acceleration
    private double m_previousSetpoint = 0;
    private double m_prevTime;

    public OnboardPositionServo(
            String name,
            VelocityMotor100<T> motor,
            Encoder100<T> encoder,
            double maxVel,
            PIDController controller,
            Profile100 profile,
            T instance) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = motor;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_profile = profile;
        m_instance = instance;
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
    public void setPosition(double goal, double feedForwardTorqueNm) {
        OptionalDouble position = m_encoder.getPosition();
        if (position.isEmpty())
            return;
        double measurement = m_instance.modulus(position.getAsDouble());

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(m_instance.modulus(goal - measurement) + measurement, 0.0);

        m_setpoint = new State100(
                m_instance.modulus(m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        final double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        final double u_FF = m_setpoint.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        final double u_TOTAL = MathUtil.clamp(u_FB + u_FF, -m_maxVel, m_maxVel);

        // pass the feedforward through unmodified
        m_motor.setVelocity(u_TOTAL, accel(u_TOTAL), feedForwardTorqueNm);
        t.logDouble(Level.DEBUG, "Desired velocity setpoint", () -> u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        t.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        t.logDouble(Level.TRACE, "u_FF", () -> u_FF);
        t.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        t.log(Level.DEBUG, "Goal", m_goal);
        t.logDouble(Level.DEBUG, "Measurement", () -> measurement);
        t.log(Level.DEBUG, "Setpoint", m_setpoint);
        t.logDouble(Level.TRACE, "Controller Position Error", () -> m_controller.getPositionError());
        t.logDouble(Level.TRACE, "Controller Velocity Error", () -> m_controller.getVelocityError());
        t.logDouble(Level.TRACE, "Feedforward Torque", () -> feedForwardTorqueNm);
    }

    /**
     * @return Current position measurement. For distance this is meters, for angle
     *         this is radians.
     */
    @Override
    public OptionalDouble getPosition() {
        OptionalDouble position = m_encoder.getPosition();
        if (position.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(m_instance.modulus(position.getAsDouble()));
    }

    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRate();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        t.logDouble(Level.DEBUG, "Position Tolerance", () -> m_controller.getPositionTolerance());
        t.logDouble(Level.DEBUG, "Velocity Tolerance", () -> m_controller.getVelocityTolerance());
        t.logBoolean(Level.DEBUG, "At Setpoint", atSetpoint);
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
    @Override
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
