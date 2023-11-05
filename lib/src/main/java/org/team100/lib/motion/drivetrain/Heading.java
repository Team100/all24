package org.team100.lib.motion.drivetrain;

import org.team100.lib.sensors.RedundantGyroInterface;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * To make sure we calculate heading the same way everywhere. This is NWU,
 * counterclockwise-positive.
 */
public class Heading implements HeadingInterface {
    private final Telemetry t = Telemetry.get();

    private final RedundantGyroInterface m_gyro;

    public Heading(RedundantGyroInterface gyro) {
        m_gyro = gyro;
    }

    @Override
    public Rotation2d getHeadingNWU() {
        double yawNED = m_gyro.getRedundantYawNED();
        // invert NED to get NWU
        Rotation2d result = Rotation2d.fromDegrees(-1.0 * yawNED);
        t.log("/Heading/Heading deg", result.getDegrees());
        t.log("/Heading/Heading rad", result.getRadians());
        return result;
    }

    @Override
    public double getHeadingRateNWU() {
        double rateNED = m_gyro.getRedundantGyroRateNED();
        // invert NED to get NWU
        return -1.0 * rateNED;
    }

}