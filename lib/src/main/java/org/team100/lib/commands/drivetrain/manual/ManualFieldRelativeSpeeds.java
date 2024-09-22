package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.FieldRelativeVelocityLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Transform manual input into a field-relative velocity.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements FieldRelativeDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final FieldRelativeVelocityLogger m_log_twist;

    public ManualFieldRelativeSpeeds(SupplierLogger2 parent, SwerveKinodynamics swerveKinodynamics) {
        SupplierLogger2 child = parent.child(this);
        m_log_twist = child.fieldRelativeVelocityLogger(Level.TRACE, "twist");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    @Override
    public FieldRelativeVelocity apply(SwerveState state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);

        // scale to max in both translation and rotation
        // and desaturate to feasibility
        final FieldRelativeVelocity twistM_S = m_swerveKinodynamics.analyticDesaturation(
                DriveUtil.scale(
                        clipped,
                        m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                        m_swerveKinodynamics.getMaxAngleSpeedRad_S()));

        m_log_twist.log(() -> twistM_S);
        return twistM_S;
    }

    @Override
    public void reset(Pose2d p) {
        //
    }

    @Override
    public String getGlassName() {
        return "ManualFieldRelativeSpeeds";
    }

}
