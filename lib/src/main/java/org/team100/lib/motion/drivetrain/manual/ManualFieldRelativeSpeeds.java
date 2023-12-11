package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Function;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Transform manual input into a field-relative Twist2d.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements Function<Twist2d, Twist2d> {
    private final Telemetry t = Telemetry.get();
    private final SpeedLimits m_speedLimits;

    public ManualFieldRelativeSpeeds(SpeedLimits speedLimits) {
        m_speedLimits = speedLimits;
    }

    @Override
    public Twist2d apply(Twist2d input) {
        Twist2d twistM_S = DriveUtil.scale(
                input,
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);
        t.log(Level.DEBUG, "/manual field relative/twist x m_s", twistM_S.dx);
        t.log(Level.DEBUG, "/manual field relative/twist y m_s", twistM_S.dy);
        t.log(Level.DEBUG, "/manual field relative/twist theta rad_s", twistM_S.dtheta);
        return twistM_S;
    }
}
