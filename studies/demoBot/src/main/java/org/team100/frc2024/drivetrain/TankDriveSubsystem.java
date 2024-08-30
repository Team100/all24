package org.team100.frc2024.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase implements Glassy{
    private final SupplierLogger m_logger;
    private final TankModuleCollection m_modules;

    public TankDriveSubsystem(
            SupplierLogger parent,
            TankModuleCollection modules) {
        m_logger = parent.child(this);
        m_modules = modules;
    }

    @Override
    public String getGlassName() {
        return "Tank Drive Subsystem";
    }

    @Override
    public void periodic() {
      m_logger.logOptionalDouble(Level.TRACE, "Left Drive Velocity M_S",() -> m_modules.getSpeeds()[0]);
      m_logger.logOptionalDouble(Level.TRACE, "Right Drive Velocity M_S",() -> m_modules.getSpeeds()[1]);

    }

    /**
     * 
     * @param translationSpeed -1 to 1 duty cycle
     * @param rotSpeed -1 to 1 duty cycle
     * 
     */
    public void set(double translationSpeed, double rotSpeed) {
      double invertedRot = rotSpeed * -1.0;
        double leftSpeed = translationSpeed + invertedRot;
        double rightSpeed = translationSpeed - invertedRot;
        setRawModules(leftSpeed * 0.5, rightSpeed* 0.5);
    }

    public void setRawModules(double... states) {
        m_modules.setDrive(states);
    }

    public void stop() {
        m_modules.stop();
    }
}
