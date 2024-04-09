package org.team100.lib.sensors;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * To make sure we calculate heading the same way everywhere. This is NWU,
 * counterclockwise-positive.
 */
public class Heading implements HeadingInterface {
    private final Telemetry t = Telemetry.get();

    private final Gyro100 m_gyro;
    private final String m_name;

    // updated by periodic.
    private Rotation2d m_currentHeadingNWU;
    private double m_currentHeadingRateNWU;

    public Heading(Gyro100 gyro) {
        m_gyro = gyro;
        m_name = Names.name(this);
        // initialize
        periodic();
        // please call periodic().
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public Rotation2d getHeadingNWU() {
        return m_currentHeadingNWU;
    }

    @Override
    public double getHeadingRateNWU() {
        return m_currentHeadingRateNWU;
    }

    @Override
    public void periodic() {
        // invert NED to get NWU
        m_currentHeadingNWU = Rotation2d.fromDegrees(-1.0 * m_gyro.getYawNEDDeg());
        m_currentHeadingRateNWU = Math.toRadians(-1.0 * m_gyro.getYawRateNEDDeg_s());
        t.log(Level.TRACE, m_name, "Heading NWU (rad)", m_currentHeadingNWU);
        t.log(Level.TRACE, m_name, "Heading Rate NWU (rad_s)", m_currentHeadingRateNWU);
    }

}