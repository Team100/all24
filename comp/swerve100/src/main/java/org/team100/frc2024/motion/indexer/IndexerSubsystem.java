package org.team100.frc2024.motion.indexer;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add indexer to selftest.
 */
public class IndexerSubsystem extends SubsystemBase implements Speeding {
    // TODO GET THE RIGHT NUMBERS
    private final String m_name;
    private final LimitedVelocityServo<Distance100> driveMotor;
    private final SpeedingVisualization m_viz;


    public IndexerSubsystem(int driveID) {
        m_name = Names.name(this);
        SysParam params = SysParam.limitedNeoVelocityServoSystem(1, .05, 8, 20, -20);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                driveMotor = ServoFactory.limitedNeoVelocityServo(
                        m_name,
                        driveID,
                        false,
                        params);
                break;
            case BLANK:
            default:
                driveMotor = ServoFactory.limitedSimulatedVelocityServo(
                        m_name,
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    public void setDrive(double value) {
        driveMotor.setDutyCycle(value);
    }
    
    @Override
    public double getVelocity() {
        return driveMotor.getVelocity();
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
        m_viz.periodic();
    }
}
