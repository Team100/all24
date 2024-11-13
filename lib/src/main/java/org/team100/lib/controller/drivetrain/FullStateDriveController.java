package org.team100.lib.controller.drivetrain;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/**
 * Three independent axes of proportional position and velocity control, with
 * setpoint velocity feedforward.
 */
public class FullStateDriveController implements HolonomicFieldRelativeController {
    private static final double kXK1 = 4; // position
    private static final double kXK2 = 0.25; // velocity
    private static final double kThetaK1 = 4; // position
    private static final double kThetaK2 = 0.25; // velocity
    private static final double kXTolerance = 0.01; // 1 cm
    private static final double kThetaTolerance = 0.02; // 1 degree
    private static final double kXDotTolerance = 0.01; // 1 cm/s
    private static final double kOmegaTolerance = 0.02; // 1 degree/s

    private final Log m_log;

    private boolean m_atSetpoint = false;

    public FullStateDriveController(Log log) {
        m_log = log;
    }

    @Override
    public FieldRelativeVelocity calculate(SwerveModel measurement, SwerveModel reference) {
        m_log.measurement.log(() -> measurement);
        m_log.reference.log(() -> reference);
        m_log.error.log(() -> reference.minus(measurement));

        FieldRelativeVelocity u_FF = reference.velocity();

        m_atSetpoint = true;

        double xFB = calculateFB(kXK1, kXK2, kXTolerance, kXDotTolerance,
                measurement.x(), reference.x(), x -> x);
        double yFB = calculateFB(kXK1, kXK2, kXTolerance, kXDotTolerance,
                measurement.y(), reference.y(), x -> x);
        double thetaFB = calculateFB(kThetaK1, kThetaK2, kThetaTolerance, kOmegaTolerance,
                measurement.theta(), reference.theta(), MathUtil::angleModulus);

        FieldRelativeVelocity u_FB = new FieldRelativeVelocity(xFB, yFB, thetaFB);
        m_log.u_FB.log(() -> u_FB);

        return u_FF.plus(u_FB);
    }

    private double calculateFB(
            double k1,
            double k2,
            double xTolerance,
            double xDotTolerance,
            Model100 measurement,
            Model100 setpoint,
            DoubleUnaryOperator modulus) {
        double xError = modulus.applyAsDouble(setpoint.x() - measurement.x());
        double xDotError = setpoint.v() - measurement.v();
        m_atSetpoint &= Math.abs(xError) < xTolerance
                && Math.abs(xDotError) < xDotTolerance;
        return k1 * xError + k2 * xDotError;
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    @Override
    public boolean atReference() {
        return m_atSetpoint;
    }

    @Override
    public void reset() {
        m_atSetpoint = false;
    }
}
