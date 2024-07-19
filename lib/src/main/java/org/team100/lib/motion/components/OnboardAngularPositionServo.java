package org.team100.lib.motion.components;

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
import edu.wpi.first.wpilibj.Timer;

/**
 * Because the 2025 angular encoder classes do not wind up, this is a version of
 * the position servo that understands that; it's almost a copy of
 * OnboardPositionServo.
 */
public class OnboardAngularPositionServo implements AngularPositionServo {
    private static final double kDtSec = 0.02;

    private final SupplierLogger m_logger;
    private final RotaryMechanism m_mechanism;
    private final RotaryPositionSensor m_encoder;
    private final double m_maxVel;
    private final PIDController m_controller;
    private final double m_period;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile = new NullProfile();

    private State100 m_goal = new State100(0, 0);
    private State100 m_setpoint = new State100(0, 0);
    // for calculating acceleration
    private double m_previousSetpoint = 0;
    private double m_prevTime;

    /** Don't forget to set a profile. */
    public OnboardAngularPositionServo(
            SupplierLogger parent,
            RotaryMechanism mech,
            RotaryPositionSensor encoder,
            double maxVel,
            PIDController controller) {
        m_logger = parent.child(this);
        m_mechanism = mech;
        m_encoder = encoder;
        m_maxVel = maxVel;
        m_controller = controller;
        m_period = controller.getPeriod();
        m_controller.setIntegratorRange(0, 0.1);
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
        m_prevTime = Timer.getFPGATimestamp();
        OptionalDouble position = getPosition();
        OptionalDouble velocity = getVelocity();
        if (position.isEmpty() || velocity.isEmpty()) {
            Util.warn("OnboardAngularPositionServo: Broken sensor!");
            return;
        }
        m_setpoint = new State100(position.getAsDouble(), velocity.getAsDouble());
    }

    @Override
    public void setProfile(Profile100 profile) {
        m_profile = profile;
    }

    @Override
    public void setPositionWithVelocity(
            double goalRad,
            double goalVelocityRad_S,
            double feedForwardTorqueNm) {
        OptionalDouble positionRad = m_encoder.getPositionRad();
        if (positionRad.isEmpty())
            return;
        double measurementRad = MathUtil.angleModulus(positionRad.getAsDouble());

        // use the modulus closest to the measurement.
        m_goal = new State100(MathUtil.angleModulus(goalRad - measurementRad) + measurementRad, goalVelocityRad_S);

        m_setpoint = new State100(
                MathUtil.angleModulus(m_setpoint.x() - measurementRad) + measurementRad,
                m_setpoint.v());

        m_setpoint = m_profile.calculate(m_period, m_setpoint, m_goal);

        final double u_FB = m_controller.calculate(measurementRad, m_setpoint.x());
        final double u_FF = m_setpoint.v();
        // note u_FF is rad/s, so a big number, u_FB should also be a big number.

        final double u_TOTAL = MathUtil.clamp(u_FB + u_FF, -m_maxVel, m_maxVel);

        m_mechanism.setVelocity(u_TOTAL, accel(u_TOTAL), feedForwardTorqueNm);

        m_logger.logState100(Level.TRACE, "goal (rad)", () -> m_goal);
        m_logger.logDouble(Level.TRACE, "Feedforward Torque (Nm)", () -> feedForwardTorqueNm);
        m_logger.logDouble(Level.TRACE, "measurement (rad)", () -> measurementRad);
        m_logger.logState100(Level.TRACE, "setpoint (rad)", () -> m_setpoint);
        m_logger.logDouble(Level.TRACE, "u_FB (rad_s)", () -> u_FB);
        m_logger.logDouble(Level.TRACE, "u_FF (rad_s)", () -> u_FF);
        m_logger.logDouble(Level.TRACE, "u_TOTAL (rad_s)", () -> u_TOTAL);
        m_logger.logDouble(Level.TRACE, "Controller Position Error (rad)", m_controller::getPositionError);
        m_logger.logDouble(Level.TRACE, "Controller Velocity Error (rad_s)", m_controller::getVelocityError);
    }

    @Override
    public void setPosition(double goalRad, double feedForwardTorqueNm) {
        setPositionWithVelocity(goalRad, 0.0, feedForwardTorqueNm);
    }

    /**
     * @return Current position measurement, radians.
     */
    @Override
    public OptionalDouble getPosition() {
        OptionalDouble position = m_encoder.getPositionRad();
        if (position.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of(MathUtil.angleModulus(position.getAsDouble()));
    }

    /**
     * @return Current velocity, rad/s.
     */
    @Override
    public OptionalDouble getVelocity() {
        return m_encoder.getRateRad_S();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        m_logger.logDouble(Level.TRACE, "Position Tolerance", m_controller::getPositionTolerance);
        m_logger.logDouble(Level.TRACE, "Velocity Tolerance", m_controller::getVelocityTolerance);
        m_logger.logBoolean(Level.TRACE, "At Setpoint", () -> atSetpoint);
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
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_encoder.close();
    }

    /** for testing only */
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
        double dt = now - m_prevTime;
        m_prevTime = now;
        double accel = (setpoint - m_previousSetpoint) / dt;
        m_previousSetpoint = setpoint;
        return accel;
    }
}
