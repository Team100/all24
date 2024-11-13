package org.team100.lib.controller.drivetrain;

import org.team100.lib.controller.simple.MinTimeController;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Control100;

import edu.wpi.first.math.MathUtil;

/** Min-time controllers on all axes. */
public class MinTimeDriveController implements HolonomicFieldRelativeController {
    /** Should be stronger than switching. */
    private static final int INITIAL_CURVE_ACCEL = 12;
    /** Should be weaker than switching. */
    private static final int GOAL_CURVE_ACCEL = 7;
    private static final int SWITCHING_CURVE_ACCEL = 9;
    private static final double[] FULL_STATE_K = new double[] { 2.0, 0.2 };
    /** Switch to full-state proportional when this close to the goal. */
    private static final double EASE = 0.1;
    private static final double TOLERANCE = 0.01;
    private static final int MAX_OMEGA_RAD_S = 5;
    private static final int MAX_VELOCITY_M_S = 5;
    private final MinTimeController m_xController;
    private final MinTimeController m_yController;
    private final MinTimeController m_thetaController;
    private final Log m_log;

    public MinTimeDriveController(LoggerFactory parent, Log log) {
        LoggerFactory child = parent.child(this);
        m_xController = new MinTimeController(
                child,
                x -> x,
                MAX_VELOCITY_M_S,
                SWITCHING_CURVE_ACCEL,
                GOAL_CURVE_ACCEL,
                INITIAL_CURVE_ACCEL,
                TOLERANCE,
                EASE,
                FULL_STATE_K);
        m_yController = new MinTimeController(
                child,
                x -> x,
                MAX_VELOCITY_M_S,
                SWITCHING_CURVE_ACCEL,
                GOAL_CURVE_ACCEL,
                INITIAL_CURVE_ACCEL,
                TOLERANCE,
                EASE,
                FULL_STATE_K);
        m_thetaController = new MinTimeController(
                child,
                MathUtil::angleModulus,
                MAX_OMEGA_RAD_S,
                SWITCHING_CURVE_ACCEL,
                GOAL_CURVE_ACCEL,
                INITIAL_CURVE_ACCEL,
                TOLERANCE,
                EASE,
                FULL_STATE_K);
        m_log = log;
    }

    @Override
    public boolean atReference() {
        return m_xController.atSetpoint() &&
                m_yController.atSetpoint() &&
                m_thetaController.atSetpoint();
    }

    /**
     * Makes no attempt to coordinate the axes or provide feasible output.
     */
    @Override
    public FieldRelativeVelocity calculate(SwerveModel measurement, SwerveModel reference) {
        m_log.measurement.log(() -> measurement);
        m_log.reference.log(() -> reference);
        m_log.error.log(() -> reference.minus(measurement));

        FieldRelativeVelocity u_FF = reference.velocity();

        Control100 xFB = m_xController.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                measurement.x(),
                reference.x());
        Control100 yFB = m_yController.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                measurement.y(),
                reference.y());
        Control100 thetaFB = m_thetaController.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                measurement.theta(),
                reference.theta());

        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(
                xFB.v(), yFB.v(), thetaFB.v());
        m_log.u_FB.log(() -> u_FB);
        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        // nothing to do
    }
}
