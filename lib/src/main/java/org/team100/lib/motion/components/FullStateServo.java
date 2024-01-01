package org.team100.lib.motion.components;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Profiled motion with feedforward and feedback on position and velocity.
 * 
 * It is essential to call reset() before first use and after disuse, to prevent
 * transients.
 */
public class FullStateServo<T extends Measure100> {
    // private static final double kDeadband = 0.03;

    private final Telemetry t = Telemetry.get();
    private final VelocityServo<T> m_servo;
    private final Encoder100<T> m_encoder;
    private final double m_maxVel;
    private final PIDController m_xController;
    private final PIDController m_vController;
    private final double m_period;
    private final String m_name;
    private final ChoosableProfile m_profile;
    private final T m_instance;

    private State100 m_setpoint;

    public FullStateServo(
            String name,
            VelocityServo<T> servo,
            Encoder100<T> encoder,
            double maxVel,
            PIDController xController,
            PIDController vController,
            ChoosableProfile profile,
            T instance) {
        m_servo = servo;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_xController = xController;
        m_vController = vController;
        m_period = xController.getPeriod();
        m_name = String.format("/full state servo %s", name);
        m_profile = profile;
        m_instance = instance;

        m_setpoint = null;
    }

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    public void reset(State100 measurement) {
        m_xController.reset();
        m_vController.reset();
        m_setpoint = measurement;
    }

    /**
     * TODO: allow nonzero goal velocity, use a State here
     * @param goal For distance, use meters, For angle, use radians.
     */
    public void setPosition(double goal) {
        if (m_setpoint == null) {
            Util.warn("No FullStateServo.m_setpoint! Call reset() before setPosition()");
            return;
        }
        double measurement = m_instance.modulus(m_encoder.getPosition());
        // make sure the goal and setpoint use the modulus that's close to the
        // measurement.
        // TODO: nonzero goal velocity
        State100 m_goal = new State100(
                m_instance.modulus(goal - measurement) + measurement,
                0.0);
        m_setpoint = new State100(
                m_instance.modulus(m_setpoint.x() - measurement) + measurement,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);
        double u_XFB = m_xController.calculate(measurement, m_setpoint.x());

        double velocityMeasurement = m_encoder.getRate();
        double u_VFB = m_vController.calculate(velocityMeasurement, m_setpoint.v());

        double u_FF = m_setpoint.v();
        double u_TOTAL = u_XFB + u_VFB + u_FF;
        // NOTE: deadband maybe bad?
        // u_TOTAL = MathUtil.applyDeadband(u_TOTAL, kDeadband, m_maxVel);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -m_maxVel, m_maxVel);
        m_servo.setVelocity(u_TOTAL);

        t.log(Level.DEBUG, m_name + "/u_XFB ", u_XFB);
        t.log(Level.DEBUG, m_name + "/u_VFB ", u_VFB);
        t.log(Level.DEBUG, m_name + "/u_FF", u_FF);
        t.log(Level.DEBUG, m_name + "/Position", getPosition());
        t.log(Level.DEBUG, m_name + "/Goal", m_goal.x());
        t.log(Level.DEBUG, m_name + "/Setpoint", m_setpoint.x());
        t.log(Level.DEBUG, m_name + "/Setpoint Velocity", m_setpoint.v());
        t.log(Level.DEBUG, m_name + "/Position Error", m_xController.getPositionError());
        t.log(Level.DEBUG, m_name + "/Velocity Error", m_vController.getPositionError());
        t.log(Level.DEBUG, m_name + "/Position Error Velocity", m_xController.getVelocityError());
        t.log(Level.DEBUG, m_name + "/Velocity Error Velocity", m_vController.getVelocityError());
        t.log(Level.DEBUG, m_name + "/Velocity", m_servo.getVelocity());
    }

    /** Direct velocity control for testing */
    public void setVelocity(double velocity) {
        m_servo.setVelocity(velocity);
    }

    /** Direct duty cycle for testing */
    public void setDutyCycle(double dutyCycle) {
        m_servo.setDutyCycle(dutyCycle);
    }

    /**
     * @return For distance this is meters, for angle this is radians.
     */
    public double getPosition() {
        return m_instance.modulus(m_encoder.getPosition());
    }

    public double getVelocity() {
        return m_servo.getVelocity();
    }

    public boolean atSetpoint() {
        return m_xController.atSetpoint() && m_vController.atSetpoint();
    }

    public void stop() {
        m_servo.stop();
    }

    public void close() {
        m_encoder.close();
    }

    public State100 getSetpoint() {
        return m_setpoint;
    }

    public void periodic() {
        m_encoder.periodic();
    }
}
