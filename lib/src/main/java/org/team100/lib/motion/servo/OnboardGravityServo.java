package org.team100.lib.motion.servo;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.profile.NullProfile;
import org.team100.lib.profile.Profile100;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
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
 * @deprecated remove this after testing
 */
@Deprecated
public class OnboardGravityServo implements GravityServoInterface {
    private static final double kFeedbackDeadbandRad_S = 0.01;
    /** Max gravity torque */
    private static final double kGravityNm = 5.0;

    /** Offset from horizontal */
    private static final double kOffsetRad = 0.0;

    private final RotaryMechanism m_mech;
    private final PIDController m_controller;
    private final RotaryPositionSensor m_encoder;
    /** Smooth out the feedback output */
    private final LinearFilter m_filter;

    // LOGGERS
    private final DoubleSupplierLogger2 m_log_u_FB;
    private final DoubleSupplierLogger2 m_log_gravity;
    private final DoubleSupplierLogger2 m_log_u_TOTAL;
    private final DoubleSupplierLogger2 m_log_Measurement_position;
    private final DoubleSupplierLogger2 m_log_Measurement_velocity;
    private final State100Logger m_log_Goal;
    private final State100Logger m_log_Setpoint;
    private final DoubleSupplierLogger2 m_log_Controller_Position_Error;
    private final DoubleSupplierLogger2 m_log_Controller_Velocity_Error;
    private final DoubleSupplierLogger2 m_log_periodic_Measurement_position;
    private final DoubleSupplierLogger2 m_log_periodic_Measurement_velocity;

    /** Profile may be updated at runtime. */
    private Profile100 m_profile = new NullProfile();
    private State100 m_setpointRad = new State100(0, 0);

    /** Remember to set a profile! */
    public OnboardGravityServo(
            RotaryMechanism motor,
            SupplierLogger2 parent,
            PIDController controller,
            RotaryPositionSensor encoder) {
        m_mech = motor;
        SupplierLogger2 child = parent.child(this);
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB");
        m_log_gravity = child.doubleLogger(Level.TRACE, "gravity T");
        m_log_u_TOTAL = child.doubleLogger(Level.TRACE, "u_TOTAL");
        m_log_Measurement_position = child.doubleLogger(Level.TRACE, "Measurement position (rad)");
        m_log_Measurement_velocity = child.doubleLogger(Level.TRACE, "Measurement velocity (rad_s)");
        m_log_Goal = child.state100Logger(Level.TRACE, "Goal (rad)");
        m_log_Setpoint = child.state100Logger(Level.TRACE, "Setpoint (rad)");
        m_log_Controller_Position_Error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_Controller_Velocity_Error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
        m_log_periodic_Measurement_position = child.doubleLogger(Level.TRACE, "periodic Measurement position (rad)");
        m_log_periodic_Measurement_velocity = child.doubleLogger(Level.TRACE,
                "periodic Measurement velocity (rad_s)");

        m_controller = controller;
        m_encoder = encoder;
        m_filter = LinearFilter.singlePoleIIR(0.02, TimedRobot100.LOOP_PERIOD_S);
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

        m_setpointRad = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpointRad, goal);

        final double u_FB = MathUtil.applyDeadband(
                m_filter.calculate(m_controller.calculate(mechanismPositionRad, m_setpointRad.x())),
                kFeedbackDeadbandRad_S,
                10);
        // rad/s
        final double gravityTorqueNm = kGravityNm * Math.cos(mechanismPositionRad + kOffsetRad);

        final double u_TOTAL = m_setpointRad.v() + u_FB;

        m_mech.setVelocity(u_TOTAL, m_setpointRad.a(), gravityTorqueNm);

        m_log_u_FB.log(() -> u_FB);
        m_log_gravity.log(() -> gravityTorqueNm);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_Measurement_position.log(() -> mechanismPositionRad);
        m_log_Measurement_velocity.log(() -> mechanismVelocityRad_S);
        m_log_Goal.log(() -> goal);
        m_log_Setpoint.log(() -> m_setpointRad);
        m_log_Controller_Position_Error.log(m_controller::getPositionError);
        m_log_Controller_Velocity_Error.log(m_controller::getVelocityError);
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

        m_log_periodic_Measurement_position.log(() -> mechanismPositionRad);
        m_log_periodic_Measurement_velocity.log(() -> mechanismVelocityRad_S);
    }

}
