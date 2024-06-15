package org.team100.lib.sensors;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * To make sure we calculate heading the same way everywhere. This is NWU,
 * counterclockwise-positive.
 */
public class Heading implements HeadingInterface {
    private final Telemetry t = Telemetry.get();

    private final Gyro100 m_gyro;
    private final String m_name;

    public Heading(Gyro100 gyro) {
        m_gyro = gyro;
        m_name = Names.name(this);
    }

    @Override
    public Rotation2d getHeadingNWU() {
        Rotation2d currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * m_gyro.getYawNEDDeg());
        t.log(Level.TRACE, m_name, "Heading NWU (rad)", currentHeadingNWU);
        return currentHeadingNWU;
    }

    @Override
    public double getHeadingRateNWU() {
        double currentHeadingRateNWU = Math.toRadians(m_gyro.getYawRateNEDDeg_s());
        t.log(Level.TRACE, m_name, "Heading Rate NWU (rad_s)", currentHeadingRateNWU);
        return currentHeadingRateNWU;
    }


}