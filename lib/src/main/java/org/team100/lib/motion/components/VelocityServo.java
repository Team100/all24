package org.team100.lib.motion.components;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * For controllers that support it, this is just a passthrough to outboard
 * closed-loop velocity control. There's also a PIDF, selected via experiment.
 */
public class VelocityServo<T> {
    private static final double kDeadband = 0.03;

    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final Motor100<T> m_motor;
    private final Encoder100<T> m_encoder;
    private final PIDController m_controller;
    private final double m_period;
    private final SimpleMotorFeedforward m_feedforward;
    private final String m_name;

    // for calculating acceleration
    private double previousSetpoint = 0;

    /**
     * @param experiments
     * @param name        may not start with slash
     * @param motor
     * @param encoder
     * @param controller
     * @param feedforward
     */
    public VelocityServo(
            Experiments experiments,
            String name,
            Motor100<T> motor,
            Encoder100<T> encoder,
            PIDController controller,
            SimpleMotorFeedforward feedforward) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_experiments = experiments;
        m_motor = motor;
        m_encoder = encoder;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_feedforward = feedforward;
        m_name = String.format("/%s/Velocity Servo", name);
    }

    /**
     * TODO: use some sort of state here, with velocity and acceleration.
     * 
     * @param setpoint velocity
     */
    public void setVelocity(Double setpoint) {
        if (Double.isNaN(setpoint))
            throw new IllegalArgumentException("setpoint is NaN");
        if (m_experiments.enabled(Experiment.UseClosedLoopVelocity)) {
            offboard(setpoint);
        } else {
            onboard(setpoint);
        }
        t.log(Level.DEBUG, m_name + "/Desired setpoint", setpoint);
        t.log(Level.DEBUG, m_name + "/Controller Setpoint", m_controller.getSetpoint());
        t.log(Level.DEBUG, m_name + "/Controller Speed Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name + "/Controller Accel Error", m_controller.getVelocityError());
    }

    /** Direct control for testing. */
    public void setDutyCycle(double dutyCycle) {
        m_motor.setDutyCycle(dutyCycle);
    }

    /** Note this can be noisy, maybe filter it. */
    public Double getVelocity() {
        return m_encoder.getRate();
    }

    public void stop() {
        m_motor.stop();
    }

    public double getDistance() {
        return m_encoder.getPosition();
    }

    ////////////////////////////////////////////////

    private void offboard(double setpoint) {
        if (Double.isNaN(setpoint))
            throw new IllegalArgumentException("setpoint is NaN");
        m_motor.setVelocity(setpoint, 0);
    }

    private void onboard(double setpoint) {
        double u_FB = m_controller.calculate(getVelocity(), setpoint);
        double u_FF = m_feedforward.calculate(setpoint, accel(setpoint));
        double u_TOTAL = u_FB + u_FF;
        u_TOTAL = MathUtil.applyDeadband(u_TOTAL, kDeadband, 1);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -1, 1);
        m_motor.setDutyCycle(u_TOTAL);
        t.log(Level.DEBUG, m_name + "/Controller Output", u_FB);
        t.log(Level.DEBUG, m_name + "/Feed Forward Output", u_FF);
        t.log(Level.DEBUG, m_name + "/Total Output", u_TOTAL);
    }

    private double accel(double setpoint) {
        double accel = (setpoint - previousSetpoint) / m_period;
        previousSetpoint = setpoint;
        return accel;
    }
}
