package org.team100.frc2024.motion.indexer;

import javax.swing.text.Position;

import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * TODO: add indexer to selftest.
 */
public class IndexerSubsystem extends SubsystemBase {

    // TODO GET THE RIGHT NUMBERS

    SysParam driveParameter;
    SysParam ampAngleParameter;


    private final String m_name;

    private final LimitedVelocityServo<Distance100> driveMotor;

   



    public IndexerSubsystem(int canID) {
        m_name = Names.name(this);

        driveParameter = SysParam.limitedNeoVelocityServoSystem(1, 1, 1, 1, 1);
                
        driveMotor = ServoFactory.limitedNeoVelocityServo(
            m_name,
            canID,
            false,
            driveParameter);

       

        

    }

    public void setDrive(double value) {
        driveMotor.setVelocity(value);
    }

  

    // public void setAmpVelocity(double value) {
    //     ampAngleMotorRight.setPosition(value);
    //     ampAngleMotorLeft.setPosition(value);

    // }

    @Override
    public void periodic() {
        driveMotor.periodic();
    }
}
