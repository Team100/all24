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
    private final SwerveKinodynamics m_speedLimits;

    public ManualFieldRelativeSpeeds(SwerveKinodynamics speedLimits) {
        m_speedLimits = speedLimits;
    }

    public Twist2d apply(Twist2d input) {
        Twist2d twistM_S = DriveUtil.scale(
                input,
                m_speedLimits.getMaxSpeedM_S(),
                m_speedLimits.getMaxAngleSpeedRad_S());
        t.log(Level.DEBUG, "/manual field relative/twist x m_s", twistM_S.dx);
        t.log(Level.DEBUG, "/manual field relative/twist y m_s", twistM_S.dy);
        t.log(Level.DEBUG, "/manual field relative/twist theta rad_s", twistM_S.dtheta);
        return twistM_S;
    }
}
