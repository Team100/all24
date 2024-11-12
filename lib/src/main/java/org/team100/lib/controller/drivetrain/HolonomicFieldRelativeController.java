package org.team100.lib.controller.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveStateLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

public interface HolonomicFieldRelativeController extends Glassy {
    /** Implementations can share log schema. */
    public static class Log {
        final SwerveStateLogger measurement;
        final SwerveStateLogger reference; // ref v is FF
        final SwerveStateLogger error;
        final FieldRelativeVelocityLogger u_FB;

        public Log(LoggerFactory parent) {
            LoggerFactory child = parent.child("HolonomicFieldRelativeController");
            reference = child.swerveStateLogger(Level.DEBUG, "reference");
            measurement = child.swerveStateLogger(Level.DEBUG, "measurement");
            error = child.swerveStateLogger(Level.DEBUG, "error");
            u_FB = child.fieldRelativeVelocityLogger(Level.DEBUG, "u_FB");
        }
    }

    /**
     * @param measurement current state in field coordinates
     * @param reference   reference state i.e. setpoint
     * @return field-relative , meters and radians per second
     */
    FieldRelativeVelocity calculate(SwerveModel measurement, SwerveState reference);

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
