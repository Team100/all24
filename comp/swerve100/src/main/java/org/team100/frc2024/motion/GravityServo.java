package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.config.SysParam;
import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;

/**
 * Implements cosine feedforward for gravity compensation, using a duty cycle
 * motor.
 */
public class GravityServo implements Glassy {
    private final Logger m_logger;
    private final DutyCycleMotor100 m_motor;
    private final SysParam m_params;
    private final PIDController m_controller;
    private final Profile100 m_profile;
    private final double m_period;
    private final RotaryPositionSensor m_encoder;
    private final double[] m_softLimits;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public GravityServo(
            DutyCycleMotor100 motor,
            Logger parent,
            SysParam params,
            PIDController controller,
            Profile100 profile,
            double period,
            RotaryPositionSensor encoder,
            double[] softLimits) {
        m_motor = motor;
        m_logger = parent.child(this);
        m_params = params;
        m_controller = controller;
        m_controller.setTolerance(0.02);
        m_profile = profile;
        m_period = period;
        m_encoder = encoder;
        m_softLimits = softLimits;
    }

    /** Zeros controller errors, sets setpoint to current position. */
    public void reset() {
        m_controller.reset();
        if (getPositionRad().isEmpty()) {
            return;
        }
        m_setpoint = new State100(getPositionRad().getAsDouble(), 0);
    }

    public OptionalDouble getPositionRad() {
        return m_encoder.getPositionRad();
    }

    public void rezero() {
        m_encoder.reset();
    }

    public OptionalDouble getRawPositionRad() {
        return m_encoder.getPositionRad();
    }

    public void setPosition(double goal) {
        OptionalDouble encoderPosition = m_encoder.getPositionRad();
        if (encoderPosition.isEmpty()) {
            return;
        }
        double measurement = encoderPosition.getAsDouble();

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(goal, 0.0);

        m_setpoint = new State100(
                (m_setpoint.x()),
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        final double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        final double u_FF = m_setpoint.v() * 0.5; // rot/s to rpm conversion

        final double gravityTorque = 0.006 * Math.cos(measurement);
        final double staticFF = getStaticFF(measurement, u_FB, u_FF);

        // final double u_TOTAL = gravityTorque + u_FF + u_FB + staticFF;
        final double u_TOTAL = gravityTorque + u_FF + u_FB;
        m_motor.setDutyCycle(u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "static FF", () -> staticFF);
        m_logger.logDouble(Level.TRACE, "gravity T", () -> gravityTorque);
        m_logger.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Measurement", () -> measurement);
        m_logger.logState100(Level.TRACE, "Goal", () -> m_goal);
        m_logger.logState100(Level.TRACE, "Setpoint", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "Setpoint Velocity", m_setpoint::v);
        m_logger.logDouble(Level.TRACE, "Controller Position Error", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error", m_controller::getVelocityError);
        m_logger.logDouble(Level.TRACE, "COOSIIINEEE", () -> Math.cos((measurement / m_params.gearRatio())));
        m_logger.logDouble(Level.TRACE, "POSE * GEAR RAT", () -> measurement / m_params.gearRatio());
        m_logger.logDouble(Level.TRACE, "ENCODEr", () -> measurement);
    }

    private double getStaticFF(double measurement, double u_FB, double u_FF) {
        double staticFF = 0.01 * Math.signum(u_FF + u_FB);

        if (Math.abs(m_goal.x() - measurement) < m_controller.getPositionTolerance()) {
            staticFF = 0;
        }
        return staticFF;
    }

    public void setWithSoftLimits(double value) {
        OptionalDouble encoderPosition = m_encoder.getPositionRad();
        if (encoderPosition.isEmpty()) {
            m_motor.setDutyCycle(0);
            return;
        }
        double measurement = encoderPosition.getAsDouble();
        if (value >= 0) {
            if (measurement >= m_softLimits[1]) {
                m_motor.setDutyCycle(0);
                return;
            }
        } else if (value <= 0 && measurement <= m_softLimits[0]) {
            m_motor.setDutyCycle(0);
            return;
        }
        m_motor.setDutyCycle(value);
    }

    public void setPositionWithSteadyState(double goal) {
        OptionalDouble encoderPosition = m_encoder.getPositionRad();
        if (encoderPosition.isEmpty()) {
            m_motor.setDutyCycle(0);
            return;
        }
        double measurement = encoderPosition.getAsDouble();

        // use the modulus closest to the measurement.
        // note zero velocity in the goal.
        m_goal = new State100(goal, 0.0);

        m_setpoint = new State100(
                (m_setpoint.x()),
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        double u_FF = m_setpoint.v() * 0.01; // rot/s to rpm conversion

        final double fudgeFactor = 0.9;
        final double gravityTorque = 0.015 * 3 * Math.cos((measurement / m_params.gearRatio())) * fudgeFactor;

        double u_TOTAL = gravityTorque + u_FF + u_FB;

        m_motor.setDutyCycle(gravityTorque + u_FF + u_FB);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "GRAVITY", () -> gravityTorque);
        m_logger.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Measurement", () -> measurement);
        m_logger.logState100(Level.TRACE, "Goal", () -> m_goal);
        m_logger.logState100(Level.TRACE, "Setpoint", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "Setpoint Velocity", () -> m_setpoint.v());
        m_logger.logDouble(Level.TRACE, "Controller Position Error", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error", m_controller::getVelocityError);
        m_logger.logDouble(Level.TRACE, "COOSIIINEEE", () -> Math.cos((measurement / m_params.gearRatio())));
        m_logger.logDouble(Level.TRACE, "POSE * GEAR RAT", () -> measurement / m_params.gearRatio());
        m_logger.logDouble(Level.TRACE, "ENCODEr", () -> measurement);
    }

    public void set(double value) {
        m_motor.setDutyCycle(value);
    }

    public void stop() {
        m_motor.setDutyCycle(0);
    }

    @Override
    public String getGlassName() {
        return "GravityServo";
    }
}
