package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

/**
 * Swerve drive motor using Talon FX and Phoenix 6.
 * 
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * Phoenix 6 uses a Kalman filter to eliminate velocity measurement lag.
 */
public abstract class Talon6DriveMotor extends Talon6Motor<Distance100> {

    private final double m_gearRatio;
    private final double m_wheelDiameterM;
    // TODO: just use distance???
    private final double m_distancePerTurn;

    protected Talon6DriveMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            double gearRatio,
            double wheelDiameter,
            PIDConstants pid,
            Feedforward100 ff) {
        super(name, canId, motorPhase, supplyLimit, statorLimit, pid, ff);
        m_gearRatio = gearRatio;
        m_wheelDiameterM = wheelDiameter;
        m_distancePerTurn = wheelDiameter * Math.PI / gearRatio;
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S_S, double outputTorqueN) {
        double wheelRev_S = outputM_S / (m_wheelDiameterM * Math.PI);
        t.logDouble(Level.TRACE, "module input (RPS)", wheelRev_S);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double wheelRev_S2 = accelM_S_S / (m_wheelDiameterM * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        double wheelTorqueNm = outputTorqueN * m_wheelDiameterM / 2;
        double motorTorqueNm = wheelTorqueNm / m_gearRatio;
        setMotorVelocity(motorRev_S, motorRev_S2, motorTorqueNm);
    }

    @Override
    public void setPosition(double outputM, double outputM_S, double outputTorqueN) {
        double wheelRev = outputM / (m_wheelDiameterM * Math.PI);
        double motorRev = wheelRev * m_gearRatio;
        double wheelRev_S = outputM_S / (m_wheelDiameterM * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double wheelTorqueNm = outputTorqueN * m_wheelDiameterM / 2;
        double motorTorqueNm = wheelTorqueNm / m_gearRatio;
        setMotorPosition(motorRev, motorRev_S, motorTorqueNm);
    }

}
