package org.team100.lib.motion.components;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * Positional control on top of a velocity servo.
 */
public class PositionServo<T> {
    // NOTE: i took out the deadband because i was looking for more accuracy,
    // but that might result in chattering, so feel free to put it back.
    // private static final double kDeadband = 0.03;

    private final Telemetry t = Telemetry.get();
    private final VelocityServo<T> m_servo;
    private final Encoder100<T> m_encoder;
    private final double m_maxVel;
    private final PIDController m_controller;
    private final double m_minimumInput;
    private final double m_maximumInput;
    private final double m_period;
    private final String m_name;
    private final ChoosableProfile m_profile;
    private final DoubleUnaryOperator m_modulus;

    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    // TODO: use a profile that exposes acceleration and use it.
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    /**
     * @param name    may not start with a slash
     * @param modulus wrap the measurement if desired
     */
    public PositionServo(
            String name,
            VelocityServo<T> servo,
            Encoder100<T> encoder,
            double maxVel,
            PIDController controller,
            ChoosableProfile profile,
            DoubleUnaryOperator modulus) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_servo = servo;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_controller = controller;
        if (m_controller.isContinuousInputEnabled()) {
            m_minimumInput = -Math.PI;
            m_maximumInput = Math.PI;
        } else {
            m_minimumInput = -Double.MAX_VALUE;
            m_maximumInput = Double.MAX_VALUE;
        }
        m_period = controller.getPeriod();
        m_name = String.format("/%s/Position Servo", name);
        m_profile = profile;
        m_modulus = modulus;
    }

    /**
     * @param goal For distance, use meters, For angle, use radians.
     */
    public void setPosition(double goal) {
        double measurement = m_modulus.applyAsDouble(m_encoder.getPosition());
        m_goal = new TrapezoidProfile.State(goal, 0.0);

        if (m_controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance = MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound,
                    errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest
            // path. The goal
            // may be outside the input range after this operation, but that's OK because
            // the controller
            // will still go there and report an error of zero. In other words, the setpoint
            // only needs to
            // be offset from the measurement by the input range modulus; they don't need to
            // be equal.
            m_goal.position = goalMinDistance + measurement;
            m_setpoint.position = setpointMinDistance + measurement;
        }

        m_setpoint = m_profile.calculate(m_period, m_goal, m_setpoint);

        double u_FB = m_controller.calculate(measurement, m_setpoint.position);
        double u_FF = m_setpoint.velocity;
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        double u_TOTAL = u_FB + u_FF;
        // NOTE: i took out the deadband because i was looking for more accuracy,
        // but that might result in chattering, so feel free to put it back.
        // u_TOTAL = MathUtil.applyDeadband(u_TOTAL, kDeadband, m_maxVel);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -m_maxVel, m_maxVel);
        m_servo.setVelocity(u_TOTAL);

        t.log(Level.DEBUG, m_name + "/u_FB", u_FB);
        t.log(Level.DEBUG, m_name + "/u_FF", u_FF);
        t.log(Level.DEBUG, m_name + "/u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name + "/Measurement", measurement);
        t.log(Level.DEBUG, m_name + "/Goal", m_goal.position);
        t.log(Level.DEBUG, m_name + "/Setpoint", m_setpoint.position);
        t.log(Level.DEBUG, m_name + "/Setpoint Velocity", m_setpoint.velocity);
        t.log(Level.DEBUG, m_name + "/Controller Position Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name + "/Controller Velocity Error", m_controller.getVelocityError());
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
        return m_modulus.applyAsDouble(m_encoder.getPosition());
    }

    public double getVelocity() {
        return m_servo.getVelocity();
    }

    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        t.log(Level.DEBUG, m_name + "/Position Tolerance", m_controller.getPositionTolerance());
        t.log(Level.DEBUG, m_name + "/Velocity Tolerance", m_controller.getVelocityTolerance());
        t.log(Level.DEBUG, m_name + "/At Setpoint", atSetpoint);
        return atSetpoint;
    }

    public boolean atGoal() {
        return atSetpoint()
                && MathUtil.isNear(
                        m_goal.position,
                        m_setpoint.position,
                        m_controller.getPositionTolerance())
                && MathUtil.isNear(
                        m_goal.velocity,
                        m_setpoint.velocity,
                        m_controller.getVelocityTolerance());
    }

    public double getGoal() {
        return m_goal.position;
    }

    public void stop() {
        m_servo.stop();
    }

    public void close() {
        m_encoder.close();
    }

    /** for testing only */
    public State getSetpoint() {
        return m_setpoint;
    }
}
