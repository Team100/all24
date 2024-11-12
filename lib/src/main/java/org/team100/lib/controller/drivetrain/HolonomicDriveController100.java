package org.team100.lib.controller.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.controller.PIDController;

/**
 * PID x, PID y, PID theta, and (optionally) PID omega.
 */
public class HolonomicDriveController100 implements HolonomicFieldRelativeController {
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final boolean m_useOmega;
    private final Log m_log;

    /**
     * Use the factory.
     * 
     * @param useOmega include omega feedback
     */
    HolonomicDriveController100(Log log, boolean useOmega) {
        m_xController = HolonomicDriveControllerFactory.cartesian();
        m_yController = HolonomicDriveControllerFactory.cartesian();
        m_thetaController = HolonomicDriveControllerFactory.theta();
        m_omegaController = HolonomicDriveControllerFactory.omega();
        m_useOmega = useOmega;
        m_log = log;
    }

    @Override
    public boolean atReference() {
        if (!m_xController.atSetpoint())
            return false;
        if (!m_yController.atSetpoint())
            return false;
        if (!m_thetaController.atSetpoint())
            return false;
        if (!m_useOmega)
            return true;
        return m_omegaController.atSetpoint();
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

        double xFB = m_xController.calculate(measurement.x().x(), reference.x().x());
        double yFB = m_yController.calculate(measurement.y().x(), reference.y().x());
        double thetaFB = m_thetaController.calculate(measurement.theta().x(), reference.theta().x());
        double omegaFB = 0.0;
        if (m_useOmega) {
            omegaFB = m_omegaController.calculate(measurement.theta().v(), reference.theta().v());
        }
        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB + omegaFB);
        m_log.u_FB.log(() -> u_FB);
        return u_FF.plus(u_FB);
    }

    @Override
    public void reset() {
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
        if (m_useOmega)
            m_omegaController.reset();
    }
}