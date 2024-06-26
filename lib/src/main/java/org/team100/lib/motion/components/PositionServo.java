package org.team100.lib.motion.components;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.Motor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Positional control.
 */
public class PositionServo<T extends Measure100> implements PositionServoInterface<T> {
    private final Telemetry t = Telemetry.get();
    private final Motor100<T> m_motor;
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
    private double previousSetpoint = 0;
    private double prevTime;

    /**
     * @param name may not start with a slash
     */
    public PositionServo(
            String name,
            Motor100<T> motor,
            Encoder100<T> encoder,
            double maxVel,
            PIDController controller,
            Profile100 profile,
            T instance) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = motor;
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
        prevTime = Timer.getFPGATimestamp();

        // TODO figure this out
        // ALERT! @joel 2/19/24: I think encoder reset changes the internal offset
        // which is never what we want. but this might be wrong
        // for some other reason
        // m_encoder.reset();
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

        m_motor.setVelocity(u_TOTAL, accel(u_TOTAL), 0);
        t.log(Level.DEBUG, m_name, "Desired velocity setpoint", u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.TRACE, m_name, "u_FB", u_FB);
        t.log(Level.TRACE, m_name, "u_FF", u_FF);
        t.log(Level.TRACE, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.TRACE, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.TRACE, m_name, "Controller Velocity Error", m_controller.getVelocityError());
    }

    /**
     * @param goal For distance, use meters, For angle, use radians.
     */
    @Override
    public void setPositionDirect(double goal) {
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

        double u_TOTAL = (u_FB + u_FF) / m_maxVel;
        u_TOTAL = MathUtil.clamp(u_TOTAL, -1, 1);

        m_motor.setDutyCycle(u_TOTAL);
        t.log(Level.DEBUG, m_name, "Desired velocity setpoint", u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.TRACE, m_name, "u_FB", u_FB);
        t.log(Level.TRACE, m_name, "u_FF", u_FF);
        t.log(Level.TRACE, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.TRACE, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.TRACE, m_name, "Controller Velocity Error", m_controller.getVelocityError());
    }

    /**
     * @param goal                For distance, use meters, For angle, use radians.
     * @param feedForwardTorqueNm used for gravity compensation, passthrough.
     */
    @Override
    public void setPosition(double goal, double feedForwardTorqueNm) {
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

        // pass the feedforward through unmodified
        m_motor.setVelocity(u_TOTAL, accel(u_TOTAL), feedForwardTorqueNm);
        t.log(Level.DEBUG, m_name, "Desired velocity setpoint", u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.TRACE, m_name, "u_FB", u_FB);
        t.log(Level.TRACE, m_name, "u_FF", u_FF);
        t.log(Level.TRACE, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.TRACE, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.TRACE, m_name, "Controller Velocity Error", m_controller.getVelocityError());
        t.log(Level.TRACE, m_name, "Feedforward Torque", feedForwardTorqueNm);
    }

    /** Direct velocity control for testing */
    @Override
    public void setVelocity(double velocity) {
        m_motor.setVelocity(velocity, accel(velocity), 0);
        t.log(Level.DEBUG, m_name, "Desired velocity setpoint", velocity);
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
        return m_encoder.getRate();
    }

    /**
     * 0-1 direct duty cycle to motor
     */
    public void set(double value) {
        m_motor.setDutyCycle(value);
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
        double dt = now - prevTime;
        prevTime = now;
        double accel = (setpoint - previousSetpoint) / dt;
        previousSetpoint = setpoint;
        return accel;
    }
}
