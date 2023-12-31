package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Transform manual input into a field-relative Twist2d.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds {
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;

    public ManualFieldRelativeSpeeds(SwerveKinodynamics swerveKinodynamics) {
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     * 
     * @param twist in control units, [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    public Twist2d apply(Twist2d input) {
        // clip the input to the unit circle
        Twist2d clipped = DriveUtil.clampTwist(input, 1.0);

        // scale to max in both translation and rotation
        Twist2d twistM_S = DriveUtil.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        twistM_S = m_swerveKinodynamics.analyticDesaturation(twistM_S);

        t.log(Level.DEBUG, "/manual field relative/twist x m_s", twistM_S.dx);
        t.log(Level.DEBUG, "/manual field relative/twist y m_s", twistM_S.dy);
        t.log(Level.DEBUG, "/manual field relative/twist theta rad_s", twistM_S.dtheta);
        return twistM_S;
    }
}
