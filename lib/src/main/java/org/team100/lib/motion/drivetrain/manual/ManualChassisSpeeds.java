package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Transform manual input into ChassisSpeeds.
 * 
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualChassisSpeeds {
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;

    public ManualChassisSpeeds(SwerveKinodynamics swerveKinodynamics) {
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     *
     * @param input in control units, [-1,1]
     * @return feasible chassis speeds in m/s and rad/s
     */
    public ChassisSpeeds apply(Twist2d input) {
        // clip the input to the unit circle
        Twist2d clipped = DriveUtil.clampTwist(input, 1.0);
        // scale to max in both translation and rotation

        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        ChassisSpeeds speeds = m_swerveKinodynamics.analyticDesaturation(scaled);

        t.log(Level.DEBUG, "/manual robot relative/vx m_s", speeds.vxMetersPerSecond);
        t.log(Level.DEBUG, "/manual robot relative/vy m_s", speeds.vyMetersPerSecond);
        t.log(Level.DEBUG, "/manual robot relative/omega rad_s", speeds.omegaRadiansPerSecond);
        return speeds;
    }
}
