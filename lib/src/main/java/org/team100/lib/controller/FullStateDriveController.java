package org.team100.lib.controller;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.State100;

import edu.wpi.first.math.MathUtil;

/**
 * Proportional position and velocity control.
 */
public class FullStateDriveController {
    private static final double kXK1 = 1;
    private static final double kXK2 = 1;
    private static final double kThetaK1 = 1;
    private static final double kThetaK2 = 1;
    private static final double kXTolerance = 0.01; // 1 cm
    private static final double kThetaTolerance = 0.02; // 1 degree
    private static final double kXDotTolerance = 0.01; // 1 cm/s
    private static final double kOmegaTolerance = 0.02; // 1 degree/s

    private boolean m_atSetpoint = false;

    public FieldRelativeVelocity calculate(SwerveState measurement, SwerveState setpoint) {
        m_atSetpoint = true;
        double dx = calculate(kXK1, kXK2, kXTolerance, kXDotTolerance,
                measurement.x(), setpoint.x(), x -> x);
        double dy = calculate(kXK1, kXK2, kXTolerance, kXDotTolerance,
                measurement.y(), setpoint.y(), x -> x);
        double dtheta = calculate(kThetaK1, kThetaK2, kThetaTolerance, kOmegaTolerance,
                measurement.theta(), setpoint.theta(), MathUtil::angleModulus);
        return new FieldRelativeVelocity(dx, dy, dtheta);
    }

    private double calculate(
            double k1,
            double k2,
            double xTolerance,
            double xDotTolerance,
            State100 measurement,
            State100 setpoint,
            DoubleUnaryOperator modulus) {
        double FF = setpoint.v();
        double xError = modulus.applyAsDouble(setpoint.x() - measurement.x());
        double xDotError = setpoint.v() - measurement.v();
        m_atSetpoint &= Math.abs(xError) < xTolerance
                && Math.abs(xDotError) < xDotTolerance;
        double FB = k1 * xError + k2 * xDotError;
        return FF + FB;
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    public boolean atReference() {
        return m_atSetpoint;
    }
}
