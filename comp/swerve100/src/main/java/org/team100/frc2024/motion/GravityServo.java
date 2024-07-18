package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;

/**
 * Implements cosine feedforward for gravity compensation, using a duty cycle
 * motor.
 * 
 * Sensor measures the mechanism (i.e. arm) 1:1.
 */
public class GravityServo implements Glassy {
    private final SupplierLogger m_logger;
    private final RotaryMechanism m_motor;
    private final PIDController m_controller;
    private final Profile100 m_profile;
    private final double m_period;
    private final RotaryPositionSensor m_encoder;

    private State100 m_setpointRad = new State100(0, 0);

    public GravityServo(
            RotaryMechanism motor,
            SupplierLogger parent,
            PIDController controller,
            Profile100 profile,
            double period,
            RotaryPositionSensor encoder) {
        m_motor = motor;
        m_logger = parent.child(this);
        m_controller = controller;
        m_controller.setTolerance(0.02);
        m_profile = profile;
        m_period = period;
        m_encoder = encoder;
    }

    /** Zeros controller errors, sets setpoint to current position. */
    public void reset() {
        m_controller.reset();
        OptionalDouble opt = getPositionRad();
        if (opt.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        m_setpointRad = new State100(opt.getAsDouble(), 0);
    }

    public OptionalDouble getPositionRad() {
        return m_encoder.getPositionRad();
    }

    public void setPosition(double goalRad) {
        OptionalDouble opt = getPositionRad();
        if (opt.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = opt.getAsDouble();

        // note zero velocity in the goal.
        State100 m_goal = new State100(goalRad, 0.0);

        m_setpointRad = m_profile.calculate(m_period, m_setpointRad, m_goal);

        final double u_FB = m_controller.calculate(mechanismPositionRad, m_setpointRad.x());
        final double u_FF = m_setpointRad.v() * 0.5;

        final double gravityTorque = 0.006 * Math.cos(mechanismPositionRad);
        final double staticFF = getStaticFF(mechanismPositionRad, goalRad, u_FB, u_FF);

        // final double u_TOTAL = gravityTorque + u_FF + u_FB + staticFF;
        final double u_TOTAL = gravityTorque + u_FF + u_FB;
        m_motor.setDutyCycle(u_TOTAL);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "static FF", () -> staticFF);
        m_logger.logDouble(Level.TRACE, "gravity T", () -> gravityTorque);
        m_logger.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Measurement (rad)", () -> mechanismPositionRad);
        m_logger.logState100(Level.TRACE, "Goal (rad)", () -> m_goal);
        m_logger.logState100(Level.TRACE, "Setpoint (rad)", () -> m_setpointRad);
        m_logger.logDouble(Level.TRACE, "Setpoint Velocity", m_setpointRad::v);
        m_logger.logDouble(Level.TRACE, "Controller Position Error (rad)", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error (rad_s)", m_controller::getVelocityError);
    }

    public void stop() {
        m_motor.stop();
    }

    @Override
    public String getGlassName() {
        return "GravityServo";
    }

    private double getStaticFF(double measurementRad, double goalRad, double u_FB, double u_FF) {
        double staticFF = 0.01 * Math.signum(u_FF + u_FB);

        if (Math.abs(goalRad - measurementRad) < m_controller.getPositionTolerance()) {
            staticFF = 0;
        }
        return staticFF;
    }

}
