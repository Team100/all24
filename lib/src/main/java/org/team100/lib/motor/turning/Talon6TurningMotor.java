package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.units.Angle100;

/**
 * Swerve steering motor using Talon FX and Phoenix 6.
 */
public abstract class Talon6TurningMotor extends Talon6Motor<Angle100> {

    private static final double kSupplyLimit = 10;
    private static final double kStatorLimit = 20;

    private final double m_gearRatio;

    protected Talon6TurningMotor(
            Logger parent,
            int canId,
            MotorPhase motorPhase,
            double gearRatio,
            PIDConstants pid,
            Feedforward100 ff) {
        super(parent, canId, motorPhase, kSupplyLimit, kStatorLimit, pid, ff);
        m_gearRatio = gearRatio;
    }

    @Override
    public void setVelocity(double outputRad_S, double accelRad_S_S, double outputTorqueNm) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
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
