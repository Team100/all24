package org.team100.lib.controller.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.SwerveStateLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;

public interface HolonomicFieldRelativeController extends Glassy {
    /** Implementations can share log schema. */
    public static class Log {
        private final SwerveStateLogger m_log_measurement;
        private final SwerveStateLogger m_log_reference;

        public Log(SupplierLogger2 parent) {
            SupplierLogger2 child = parent.child("HolonomicFieldRelativeController");

            m_log_reference = child.swerveStateLogger(Level.DEBUG, "reference");
            m_log_measurement = child.swerveStateLogger(Level.DEBUG, "measurement");

        }
    }

    /**
     * @param measurement current state in field coordinates
     * @param reference   reference state i.e. setpoint
     * @return field-relative , meters and radians per second
     */
    FieldRelativeVelocity calculate(SwerveState measurement, SwerveState reference);

    /**
     * This uses the tolerances in the controllers.
     * 
     * @return True if the pose error is within tolerance of the reference.
     */
    boolean atReference();

    /**
     * Reset controller state, e.g. velocity error.
     */
    void reset();

}
