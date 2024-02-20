package org.team100.lib.motion.drivetrain.manual;

import org.team100.lib.commands.drivetrain.ChassisSpeedDriver;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Transform manual input into ChassisSpeeds.
 * 
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualChassisSpeeds implements ChassisSpeedDriver{
    private final Telemetry t = Telemetry.get();
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final String m_name;

    public ManualChassisSpeeds(String parent, SwerveKinodynamics swerveKinodynamics) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_name = Names.append(parent, this);
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    public ChassisSpeeds apply(SwerveState state, Twist2d input) {
        // clip the input to the unit circle
        Twist2d clipped = DriveUtil.clampTwist(input, 1.0);
        // scale to max in both translation and rotation

        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        ChassisSpeeds speeds = m_swerveKinodynamics.analyticDesaturation(scaled);
        t.log(Level.TRACE, m_name, "speeds", speeds);
        return speeds;
    }

    public void reset(Pose2d p) {
        //
    }
}
