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

    private final RedundantGyroInterface m_gyro;
    private final String m_name;

    public Heading(RedundantGyroInterface gyro) {
        m_gyro = gyro;
        m_name = Names.name(this);
    }

    @Override
    public Rotation2d getHeadingNWU() {
        double yawNED = m_gyro.getRedundantYawNED();
        // invert NED to get NWU
        Rotation2d result = Rotation2d.fromDegrees(-1.0 * yawNED);
        t.log(Level.DEBUG, m_name, "heading (deg)", result.getDegrees());
        t.log(Level.DEBUG, m_name, "Heading (rad)", result.getRadians());
        return result;
    }

    @Override
    public double getHeadingRateNWU() {
        double rateNED = m_gyro.getRedundantGyroRateNED();
        // invert NED to get NWU
        return -1.0 * rateNED;
    }

    @Override
    public void periodic(){
        t.log(Level.DEBUG, m_name, "Heading NWU", getHeadingNWU());
        t.log(Level.DEBUG, m_name, "Heading Rate NWU", getHeadingRateNWU());
    }

}