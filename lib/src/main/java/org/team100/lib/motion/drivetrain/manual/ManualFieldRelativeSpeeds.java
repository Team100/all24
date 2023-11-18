package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Transform manual input into a field-relative Twist2d.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements Supplier<Twist2d> {
    private final Telemetry t = Telemetry.get();
    private final Supplier<Twist2d> m_input;
    private final SpeedLimits m_speedLimits;

    public ManualFieldRelativeSpeeds(Supplier<Twist2d> input, SpeedLimits speedLimits) {
        m_input = input;
        m_speedLimits = speedLimits;
    }

    @Override
    public Twist2d get() {
        Twist2d input = m_input.get();
        Twist2d twistM_S = DriveUtil.scale(
                input,
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);
        t.log("/manual field relative/twist x m_s", twistM_S.dx);
        t.log("/manual field relative/twist y m_s", twistM_S.dy);
        return twistM_S;
    }
}
