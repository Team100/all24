package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.profile.NullProfile;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Implements cosine feedforward for gravity compensation, using motor velocity
 * output.
 * 
 * Sensor measures the mechanism (i.e. arm) 1:1.
 * 
 * Note there is no "friction" term here since it uses the motor velocity
 * setter.
 */
public class GravityServo implements GravityServoInterface {
    private static final double kFeedbackDeadbandRad_S = 0.01;
    /** Max gravity torque, "" */
    private static final double kGravityNm = 5.0;

    /** Offset from horizontal */
    private static final double kOffsetRad = 0.0;

    private final SupplierLogger m_logger;
    private final RotaryMechanism m_mech;
    private final PIDController m_controller;
    private final double m_period;
    private final RotaryPositionSensor m_encoder;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile = new NullProfile();
    private State100 m_setpointRad = new State100(0, 0);

    /** Smooth out the feedback output */
    private LinearFilter m_filter = LinearFilter.singlePoleIIR(0.02, 0.02);

    /** Remember to set a profile! */
    public GravityServo(
            RotaryMechanism motor,
            SupplierLogger parent,
            PIDController controller,
            double period,
            RotaryPositionSensor encoder) {
        m_mech = motor;
        m_logger = parent.child(this);
        m_controller = controller;
        m_controller.setTolerance(0.02);
        m_period = period;
        m_encoder = encoder;
    }

    /** Zeros controller errors, sets setpoint to current state. */
    @Override
    public void reset() {
        m_controller.reset();
        OptionalDouble optPos = getPositionRad();
        OptionalDouble optVel = getVelocityRad_S();
        if (optPos.isEmpty() || optVel.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        m_setpointRad = new State100(optPos.getAsDouble(), optVel.getAsDouble());
    }

    @Override
    public OptionalDouble getPositionRad() {
        return m_encoder.getPositionRad();
    }

    public OptionalDouble getVelocityRad_S() {
        return m_mech.getVelocityRad_S();
    }

    /** set position with zero velocity */
    @Override
    public void setPosition(double goalRad) {
        setState(new State100(goalRad, 0));
    }

    /** allow moving end-state */
    @Override
    public void setState(State100 goal) {
        OptionalDouble optPos = getPositionRad();
        OptionalDouble optVel = getVelocityRad_S();
        if (optPos.isEmpty() || optVel.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        double mechanismVelocityRad_S = optVel.getAsDouble();

        m_setpointRad = m_profile.calculate(m_period, m_setpointRad, goal);

        final double u_FB = MathUtil.applyDeadband(
                m_filter.calculate(m_controller.calculate(mechanismPositionRad, m_setpointRad.x())),
                kFeedbackDeadbandRad_S,
                10);
        // rad/s
        final double gravityTorqueNm = kGravityNm * Math.cos(mechanismPositionRad + kOffsetRad);

        final double u_TOTAL = m_setpointRad.v() + u_FB;

        m_mech.setVelocity(u_TOTAL, m_setpointRad.a(), gravityTorqueNm);

        m_controller.setIntegratorRange(0, 0.1);

        m_logger.logDouble(Level.TRACE, "u_FB", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "gravity T", () -> gravityTorqueNm);
        m_logger.logDouble(Level.TRACE, "u_TOTAL", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Measurement position (rad)", () -> mechanismPositionRad);
        m_logger.logDouble(Level.TRACE, "Measurement velocity (rad_s)", () -> mechanismVelocityRad_S);
        m_logger.logState100(Level.TRACE, "Goal (rad)", () -> goal);
        m_logger.logState100(Level.TRACE, "Setpoint (rad)", () -> m_setpointRad);
        m_logger.logDouble(Level.TRACE, "Setpoint Velocity", m_setpointRad::v);
        m_logger.logDouble(Level.TRACE, "Controller Position Error (rad)", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error (rad_s)", m_controller::getVelocityError);
    }

    @Override
    public void stop() {
        m_mech.stop();
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_profile = profile;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mech.setTorqueLimit(torqueNm);
    }

    /** Do some logging */
    @Override
    public void periodic() {
        m_mech.periodic();
        OptionalDouble optPos = getPositionRad();
        OptionalDouble optVel = getVelocityRad_S();
        if (optPos.isEmpty() || optVel.isEmpty()) {
            Util.warn("GravityServo: Broken sensor!");
            return;
        }
        double mechanismPositionRad = optPos.getAsDouble();
        double mechanismVelocityRad_S = optVel.getAsDouble();

        m_logger.logDouble(Level.TRACE, "Measurement (rad)", () -> mechanismPositionRad);
        m_logger.logDouble(Level.TRACE, "Measurement velocity (rad_s)", () -> mechanismVelocityRad_S);
    }

}
