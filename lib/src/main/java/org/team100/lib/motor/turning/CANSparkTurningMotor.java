package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Angle100;

import com.revrobotics.CANSparkBase;

/**
 * Anglular velocity motor using REV Neo or Neo Vortex.
 */
public abstract class CANSparkTurningMotor extends CANSparkMotor<Angle100> {
    private final double m_gearRatio;

    CANSparkTurningMotor(
            Logger parent,
            CANSparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            Feedforward100 ff,
            PIDConstants pid) {
        super(parent, motor, motorPhase, currentLimit, ff, pid);
        m_gearRatio = gearRatio;
    }

    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2, double outputTorqueNm) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double outputRev_S2 = accelRad_S2 / (2 * Math.PI);
        double motorRev_S2 = outputRev_S2 * m_gearRatio;
        double motorTorqueNm = outputTorqueNm / m_gearRatio;
        setMotorVelocity(motorRev_S, motorRev_S2, motorTorqueNm);
    }

    @Override
    public void setPosition(double outputRad, double outputRad_S, double outputTorqueNm) {
        double outputRev = outputRad / (2 * Math.PI);
        double motorRev = outputRev * m_gearRatio;
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorTorqueNm = outputTorqueNm / m_gearRatio;
        setMotorPosition(motorRev, motorRev_S, motorTorqueNm);
    }
}