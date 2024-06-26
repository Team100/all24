package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.units.Distance100;

import com.revrobotics.CANSparkBase;

/**
 * Linear velocity motor using REV Neo or Neo Vortex.
 */
public abstract class CANSparkDriveMotor extends CANSparkMotor<Distance100> {
    private final double m_gearRatio;
    private final double m_wheelDiameterM;

    CANSparkDriveMotor(
            String name,
            CANSparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameterM,
            Feedforward100 ff,
            PIDConstants pid) {
        super(name, motor, motorPhase, currentLimit, ff, pid);
        m_gearRatio = gearRatio;
        m_wheelDiameterM = wheelDiameterM;
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S2, double outputTorqueN) {
        double wheelRev_S = outputM_S / (m_wheelDiameterM * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double wheelRev_S2 = accelM_S2 / (m_wheelDiameterM * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        double wheelTorqueNm = outputTorqueN * m_wheelDiameterM / 2;
        double motorTorqueNm = wheelTorqueNm / m_gearRatio;
        setMotorVelocity(motorRev_S, motorRev_S2, motorTorqueNm);
    }
}
