package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.config.SysParam;
import org.team100.lib.controller.State100;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

public class GravityServo {
    private final Telemetry t = Telemetry.get();
    private final DutyCycleMotor100 m_motor;
    private final String m_name;
    private final SysParam m_params;
    private final PIDController m_controller;
    private final Profile100 m_profile;
    private final double m_period;
    private final Encoder100<Distance100> m_encoder;
    private final double[] m_softLimits;

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);

    public GravityServo(
            DutyCycleMotor100 motor,
            String name,
            SysParam params,
            PIDController controller,
            Profile100 profile,
            double period,
            Encoder100<Distance100> encoder,
            double[] softLimits) {
        m_motor = motor;
        m_name = name;
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
        if (getPosition().isEmpty()) {
            return;
        }
        m_setpoint = new State100(getPosition().getAsDouble(), 0);
    }

    public OptionalDouble getPosition() {
        return m_encoder.getPosition();
    }

    public void rezero() {
        m_encoder.reset();
    }

    public OptionalDouble getRawPosition() {
        return m_encoder.getPosition();
    }

    public void setPosition(double goal) {
        OptionalDouble encoderPosition = m_encoder.getPosition();
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

        double u_FB = m_controller.calculate(measurement, m_setpoint.x());
        double u_FF = m_setpoint.v() * 0.5; // rot/s to rpm conversion

        double gravityTorque = 0.006 * Math.cos(measurement);

        double staticFF = 0.01 * Math.signum(u_FF + u_FB);

        if (Math.abs(m_goal.x() - measurement) < m_controller.getPositionTolerance()) {
            staticFF = 0;
        }

        // double u_TOTAL = gravityTorque + u_FF + u_FB + staticFF;
        double u_TOTAL = gravityTorque + u_FF + u_FB;
        m_motor.setDutyCycle(u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.DEBUG, m_name, "u_FB", u_FB);
        t.log(Level.DEBUG, m_name, "u_FF", u_FF);
        t.log(Level.DEBUG, m_name, "static FF", staticFF);
        t.log(Level.DEBUG, m_name, "gravity T", gravityTorque);
        t.log(Level.DEBUG, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.DEBUG, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name, "Controller Velocity Error", m_controller.getVelocityError());
        t.log(Level.DEBUG, m_name, "COOSIIINEEE", Math.cos((measurement / m_params.gearRatio())));
        t.log(Level.DEBUG, m_name, "POSE * GEAR RAT", measurement / m_params.gearRatio());
        t.log(Level.DEBUG, m_name, "ENCODEr", measurement);
    }

    public void setWithSoftLimits(double value) {
        OptionalDouble encoderPosition = m_encoder.getPosition();
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
        OptionalDouble encoderPosition = m_encoder.getPosition();
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

        double gravityTorque = 0.015 * 3 * Math.cos((measurement / m_params.gearRatio()));
        gravityTorque = gravityTorque * 0.9;
        double u_TOTAL = gravityTorque + u_FF + u_FB;

        m_motor.setDutyCycle(gravityTorque + u_FF + u_FB);

        m_controller.setIntegratorRange(0, 0.1);

        t.log(Level.DEBUG, m_name, "u_FB", u_FB);
        t.log(Level.DEBUG, m_name, "u_FF", u_FF);
        t.log(Level.DEBUG, m_name, "GRAVITY", gravityTorque);
        t.log(Level.DEBUG, m_name, "u_TOTAL", u_TOTAL);
        t.log(Level.DEBUG, m_name, "Measurement", measurement);
        t.log(Level.DEBUG, m_name, "Goal", m_goal);
        t.log(Level.DEBUG, m_name, "Setpoint", m_setpoint);
        t.log(Level.DEBUG, m_name, "Setpoint Velocity", m_setpoint.v());
        t.log(Level.DEBUG, m_name, "Controller Position Error", m_controller.getPositionError());
        t.log(Level.DEBUG, m_name, "Controller Velocity Error", m_controller.getVelocityError());
        t.log(Level.DEBUG, m_name, "COOSIIINEEE", Math.cos((measurement / m_params.gearRatio())));
        t.log(Level.DEBUG, m_name, "POSE * GEAR RAT", measurement / m_params.gearRatio());
        t.log(Level.DEBUG, m_name, "ENCODEr", measurement);
    }

    public void set(double value) {
        m_motor.setDutyCycle(value);
    }

    public void stop() {
        m_motor.setDutyCycle(0);
    }
}
