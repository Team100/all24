package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Function;

import org.team100.lib.motion.drivetrain.SpeedLimits;
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
public class ManualChassisSpeeds implements Function<Twist2d, ChassisSpeeds> {
    private final Telemetry t = Telemetry.get();
    private final SpeedLimits m_speedLimits;

    public ManualChassisSpeeds(SpeedLimits speedLimits) {
        m_speedLimits = speedLimits;
    }

    @Override
    public ChassisSpeeds apply(Twist2d input) {
        ChassisSpeeds speeds = DriveUtil.scaleChassisSpeeds(
                input,
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);
        t.log(Level.DEBUG, "/manual robot relative/vx m_s", speeds.vxMetersPerSecond);
        t.log(Level.DEBUG, "/manual robot relative/vy m_s", speeds.vyMetersPerSecond);
        return speeds;
    }
}
