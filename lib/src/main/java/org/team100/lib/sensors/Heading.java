package org.team100.lib.sensors;

import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * To make sure we calculate heading the same way everywhere. This is NWU,
 * counterclockwise-positive.
 */
public class Heading implements HeadingInterface {
    private final SupplierLogger m_logger;

    private final Gyro100 m_gyro;

    public Heading(SupplierLogger parent, Gyro100 gyro) {
        m_gyro = gyro;
        m_logger = parent.child(this);
    }

    @Override
    public Rotation2d getHeadingNWU() {
        Rotation2d currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * m_gyro.getYawNEDDeg());
        m_logger.logDouble(Level.TRACE, "Heading NWU (rad)", currentHeadingNWU::getRadians);
        return currentHeadingNWU;
    }

    @Override
    public double getHeadingRateNWU() {
        double currentHeadingRateNWU = Math.toRadians(m_gyro.getYawRateNEDDeg_s());
        m_logger.logDouble(Level.TRACE, "Heading Rate NWU (rad_s)", () -> currentHeadingRateNWU);
        return currentHeadingRateNWU;
    }

}