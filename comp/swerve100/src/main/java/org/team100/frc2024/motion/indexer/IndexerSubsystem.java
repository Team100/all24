package org.team100.frc2024.motion.indexer;

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
    SysParam driveParameter;
    SysParam ampAngleParameter;
    private final String m_name;

    private final LimitedVelocityServo<Distance100> driveMotor;

    private final PositionServo<Angle100> ampAngleMotor1;
    private final PositionServo<Angle100> ampAngleMotor2;



    public IndexerSubsystem(int canID, int canID2, int canID3) {
        m_name = Names.name(this);

        driveParameter.setkGearRatio(1);
        driveParameter.setkWheelDiameter(1);
        driveParameter.setkMaxVelocity(1);
        driveParameter.setkMaxDeccel(1);

        ampAngleParameter.setkGearRatio(15);
        ampAngleParameter.setkWheelDiameter(1);
        ampAngleParameter.setkMaxVelocity(1);
        ampAngleParameter.setkMaxDeccel(1);
        
        driveMotor = ServoFactory.limitedNeoVelocityServo(
                m_name,
                canID,
                false,
                driveParameter);

        ampAngleMotor1 = ServoFactory.neoPositionServo(m_name, canID2, false, ampAngleParameter, new PIDController(1, 0, 0));
        ampAngleMotor2 = ServoFactory.neoPositionServo(m_name, canID3, true, ampAngleParameter, new PIDController(1, 0, 0));

    }

    public void setDrive(double value) {
        driveMotor.setVelocity(value);
    }

    public void setAmpPosition(double value) {
        ampAngleMotor1.setPosition(value);
        ampAngleMotor2.setPosition(value);
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
    }
}
