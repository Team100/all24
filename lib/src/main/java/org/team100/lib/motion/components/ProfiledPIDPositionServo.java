package org.team100.lib.motion.components;

import org.team100.lib.encoder.Encoder100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.drive.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Position control using Double measuring meters/sec.
 * 
 * This would be useful for, say, an elevator.
 * 
 * TODO: use WPILib Distance Measure in 2024.
 */
public class ProfiledPIDPositionServo implements PositionServo<Double> {
    public static class Config {
        public double kDeadband = 0.03;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final Motor100<Distance> m_motor;
    private final Encoder100<Distance> m_encoder;
    private final ProfiledPIDController m_controller;
    private final SimpleMotorFeedforward m_feedforward;
    private final String m_name;

    public ProfiledPIDPositionServo(
            Experiments experiments,
            String name,
            Motor100<Distance> motor,
            Encoder100<Distance> encoder,
            ProfiledPIDController controller,
            SimpleMotorFeedforward feedforward) {
        m_experiments = experiments;
        m_motor = motor;
        m_encoder = encoder;
        m_controller = controller;
        m_feedforward = feedforward;
        m_name = String.format("/position servo %s", name);
    }

    @Override
    public void setPosition(Double value) {
        if (m_experiments.enabled(Experiment.UseClosedLoopVelocity)) {
            offboard(value);
        } else {
            onboard(value);
        }
        log();
    }

    @Override
    public Double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_motor.setDutyCycle(0);
    }

    /////////////////////////////////////////////

    private void offboard(Double goal) {
        double measurement = getPosition();
        double u_FB = m_controller.calculate(measurement, goal);
        double u_FF = getSetpointVelocity();
        double u_TOTAL = u_FB + u_FF;
        u_TOTAL = MathUtil.applyDeadband(u_TOTAL, m_config.kDeadband, m_controller.getConstraints().maxVelocity);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -m_controller.getConstraints().maxVelocity, m_controller.getConstraints().maxVelocity);
        m_motor.setVelocity(u_TOTAL, 0);

        t.log(Level.DEBUG, m_name + "/goal", goal);
        t.log(Level.DEBUG, m_name + "/measurement", measurement);
        t.log(Level.DEBUG, m_name + "/u FB", u_FB);
        t.log(Level.DEBUG, m_name + "/u FF ", u_FF);
        t.log(Level.DEBUG, m_name + "/u_TOTAL", u_TOTAL);
    }

    private void onboard(Double goal) {
        double measurement = getPosition();
        double u_FB = m_controller.calculate(measurement, goal);
        double u_FF = m_feedforward.calculate(getSetpointVelocity(), 0);
        double u_TOTAL = u_FB + u_FF;
        u_TOTAL = MathUtil.applyDeadband(u_TOTAL, m_config.kDeadband, 1);
        u_TOTAL = MathUtil.clamp(u_TOTAL, -1, 1);
        m_motor.setDutyCycle(u_TOTAL);

        t.log(Level.DEBUG, m_name + "/goal", goal);
        t.log(Level.DEBUG, m_name + "/measurement", measurement);
        t.log(Level.DEBUG, m_name + "/u_FB", u_FB);
        t.log(Level.DEBUG, m_name + "/u_FF", u_FF);
        t.log(Level.DEBUG, m_name + "/u_TOTAL", u_TOTAL);
    }

    private double getSetpointVelocity() {
        return m_controller.getSetpoint().velocity;
    }

    private void log() {
        t.log(Level.DEBUG, m_name + "/controller goal", m_controller.getGoal().position);
        t.log(Level.DEBUG, m_name + "/setpoint position", m_controller.getSetpoint().position);
        t.log(Level.DEBUG, m_name + "/setpoint velocity", m_controller.getSetpoint().velocity);
        t.log(Level.DEBUG, m_name + "/position error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name + "/velocity error", m_controller.getVelocityError());
        t.log(Level.DEBUG, m_name + "/duty cycle", m_motor.get());
    }
}
