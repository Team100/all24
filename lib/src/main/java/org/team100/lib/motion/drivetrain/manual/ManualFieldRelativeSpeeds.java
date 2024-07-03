package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.commands.drivetrain.FieldRelativeDriver;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Transform manual input into a field-relative velocity.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements FieldRelativeDriver {
    private final Telemetry.Logger t;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final String m_name;

    public ManualFieldRelativeSpeeds(String parent, SwerveKinodynamics swerveKinodynamics) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_name = Names.append(parent, this);
        t = Telemetry.get().logger(m_name);
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

        t.logDouble(Level.TRACE, "twist x m_s", () -> twistM_S.x());
        t.logDouble(Level.TRACE, "twist y m_s", () -> twistM_S.y());
        t.logDouble(Level.TRACE, "twist theta rad_s", () -> twistM_S.theta());
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
