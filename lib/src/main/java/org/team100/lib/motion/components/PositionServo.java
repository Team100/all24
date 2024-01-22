package org.team100.lib.motion.components;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Positional control on top of a velocity servo.
 */
public class PositionServo<T extends Measure100> implements PositionServoInterface<T> {
    private final Telemetry t = Telemetry.get();
    private final VelocityServo<T> m_servo;
    private final Encoder100<T> m_encoder;
    private final double m_maxVel;
    private final PIDController m_controller;
    private final double m_period;
    private final String m_name;
    private final Profile100 m_profile;
    private final T m_instance;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    /**
     * @param name may not start with a slash
     */
    public PositionServo(
            String name,
            VelocityServo<T> servo,
            Encoder100<T> encoder,
            double maxVel,
            PIDController controller,
            Profile100 profile,
            T instance) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_servo = servo;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_name = Names.append(name, this);
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
        m_setpoint = new State100(getPosition(), getVelocity());
    }

    /**
     * @param goal For distance, use meters, For angle, use radians.
     */
    @Override
    public void setPosition(double goal) {
        double measurement = m_instance.modulus(m_encoder.getPosition());

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(m_instance.modulus(goal - measurement) + measurement, 0.0);

        m_setpoint = new State100(
                m_instance.modulus(m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        double u_FF = m_setpoint.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        double u_TOTAL = u_FB + u_FF;
        u_TOTAL = MathUtil.clamp(u_TOTAL, -m_maxVel, m_maxVel);
        m_servo.setVelocity(u_TOTAL);

        t.log(Level.DEBUG, m_name, "u_FB", u_FB);
        t.log(Level.DEBUG, m_name, "u_FF", u_FF);
        t.log(Level.DEBUG, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.DEBUG, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name, "Controller Velocity Error", m_controller.getVelocityError());
    }

    /** Direct velocity control for testing */
    @Override
    public void setVelocity(double velocity) {
        // m_velocitySetpoint = m_profile.calculate(0.02, m_velocitySetpoint, new
        // State100(velocity, 0, 0));
        m_servo.setVelocity(velocity);
    }

    /** Direct duty cycle for testing */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_servo.setDutyCycle(dutyCycle);
    }

    /**
     * @return Current position measurement. For distance this is meters, for angle
     *         this is radians.
     */
    @Override
    public double getPosition() {
        return m_instance.modulus(m_encoder.getPosition());
    }

    @Override
    public double getVelocity() {
        return m_servo.getVelocity();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        t.log(Level.DEBUG, m_name, "Position Tolerance", m_controller.getPositionTolerance());
        t.log(Level.DEBUG, m_name, "Velocity Tolerance", m_controller.getVelocityTolerance());
        t.log(Level.DEBUG, m_name, "At Setpoint", atSetpoint);
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
        m_servo.stop();
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

    @Override
    public void periodic() {
        m_encoder.periodic();
        m_servo.periodic();
    }
}
