package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Transform manual input into ChassisSpeeds.
 * 
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualChassisSpeeds implements ChassisSpeedDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final ChassisSpeedsLogger m_log_chassis_speeds;

    public ManualChassisSpeeds(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory child = parent.child(this);
        m_log_chassis_speeds = child.chassisSpeedsLogger(Level.TRACE, "chassis speeds");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds, and then desaturates to a feasible holonomic velocity.
     */
    @Override
    public ChassisSpeeds apply(SwerveModel state, DriverControl.Velocity input) {
        // clip the input to the unit circle
        DriverControl.Velocity clipped = DriveUtil.clampTwist(input, 1.0);
        // scale to max in both translation and rotation

        ChassisSpeeds scaled = DriveUtil.scaleChassisSpeeds(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        // desaturate to feasibility
        ChassisSpeeds speeds = m_swerveKinodynamics.analyticDesaturation(scaled);
        m_log_chassis_speeds.log(() -> speeds);
        return speeds;
    }

    public void reset(SwerveModel p) {
        //
    }
}
