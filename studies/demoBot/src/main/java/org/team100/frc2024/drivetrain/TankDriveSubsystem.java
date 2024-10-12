package org.team100.frc2024.drivetrain;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase implements Glassy {
    private final OptionalDoubleLogger m_leftlogger;
    private final OptionalDoubleLogger m_rightlogger;
    private final TankModuleCollection m_modules;
    private final double kMaxSpeedM_S = 0.4;

    public TankDriveSubsystem(
            LoggerFactory parent,
            TankModuleCollection modules) {
        LoggerFactory logger = parent.child(this);
        m_leftlogger = logger.optionalDoubleLogger(Level.TRACE, "Left Drive Velocity M_S");
        m_rightlogger = logger.optionalDoubleLogger(Level.TRACE, "Right Drive Velocity M_S");
        m_modules = modules;
    }

    @Override
    public String getGlassName() {
        return "Tank Drive Subsystem";
    }

    @Override
    public void periodic() {
        m_leftlogger.log(() -> m_modules.getSpeeds()[0]);
        m_rightlogger.log(() -> m_modules.getSpeeds()[1]);
    }

    /**
     * 
     * @param translationSpeed -1 to 1 duty cycle
     * @param rotSpeed         -1 to 1 duty cycle
     * 
     */
    public void set(double translationSpeed, double rotSpeed) {
        double invertedRot = rotSpeed * -1.0;
        double leftSpeed = translationSpeed + invertedRot;
        double rightSpeed = translationSpeed - invertedRot;
        setRawModules(leftSpeed * kMaxSpeedM_S, rightSpeed * kMaxSpeedM_S);
    }

    public void setRawModules(double... states) {
        m_modules.setDrive(states);
    }

    public void stop() {
        m_modules.stop();
    }
}
