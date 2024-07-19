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
 * Implements cosine feedforward for gravity compensation, using motor velocity
 * output.
 * 
 * Sensor measures the mechanism (i.e. arm) 1:1.
 * 
 * Note there is no "friction" term here since it uses the motor velocity
 * setter.
 */
public class GravityServo implements Glassy {
    /** Max gravity torque, "" */
    private static final double kGravityNm = 5.0;

    /** Offset from horizontal */
    private static final double kOffsetRad = 0.0;

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
        // rad/s
        final double u_FF = m_setpointRad.v();

        final double gravityTorque = kGravityNm * Math.cos(mechanismPositionRad + kOffsetRad);

        final double u_TOTAL = u_FF + u_FB;

        m_motor.setVelocity(u_TOTAL, 0, gravityTorque);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "goal (rad)", () -> goalRad);
        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF", () -> u_FF);
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

    public void periodic() {
        OptionalDouble opt = getPositionRad();
        if (opt.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = opt.getAsDouble();
        m_logger.logDouble(Level.TRACE, "Measurement (rad)", () -> mechanismPositionRad);
    }

    @Override
    public String getGlassName() {
        return "GravityServo";
    }
}
