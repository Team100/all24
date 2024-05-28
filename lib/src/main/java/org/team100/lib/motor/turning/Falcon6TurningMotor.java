package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Phoenix100;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;

import com.ctre.phoenix6.controls.VelocityVoltage;

/**
 * PHOENIX 6 VERSION
 * 
 * Swerve steering motor using Falcon 500.
 * 
 * See {@link FalconDriveMotor} for configuration details.
 */
public class Falcon6TurningMotor extends Falcon6Motor<Angle100> {
    /**
     * Motor resistance, Kraken. Falcon is 0.03.
     * https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
     */
    private static final double kROhms = 0.025;

    @Override
    protected double kROhms() {
        // kraken
        return 0.025;
    }

    /**
     * Motor torque constant, Kraken. Falcon is 0.018.
     * https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
     */
    private static final double kTNm_amp = 0.019;

    @Override
    protected double kTNm_amp() {
        // kraken
        return 0.019;
    }

    private static final double kSupplyLimit = 10;
    private static final double kStatorLimit = 20;

    private final double m_gearRatio;

    public Falcon6TurningMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double kGearRatio,
            PIDConstants lowLevelVelocityConstants,
            Feedforward100 ff) {
        super(name, canId, motorPhase, kSupplyLimit, kStatorLimit, lowLevelVelocityConstants, ff);

        m_gearRatio = kGearRatio;

    }

    public void setVelocity(double outputRad_S, double accelRad_S_S) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double currentMotorRev_S = m_motor.getVelocity().getValueAsDouble();
        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts;

        VelocityVoltage v = new VelocityVoltage(motorRev_S);
        v.FeedForward = kFFVolts;
        v.Acceleration = motorRev_S2;
        Phoenix100.warn(() -> m_motor.setControl(v));

        t.log(Level.TRACE, m_name, "motor input (RPS)", motorRev_S);
        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        log();

    }

    public void setVelocity(double outputRad_S, double accelRad_S_S, double torqueNm) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        setMotorVelocity(motorRev_S, motorRev_S2, torqueNm);
    }

    /** Position in rad */
    @Override
    public Double getPosition() {
        double positionRev = m_motor.getPosition().getValueAsDouble();
        double positionRad = positionRev * 2 * Math.PI;
        t.log(Level.TRACE, m_name, "position (rev)", positionRev);
        t.log(Level.DEBUG, m_name, "position (rad)", positionRad);
        return positionRad;
    }

    /** Velocity in rad/sec */
    @Override
    public double getRate() {
        double velocityRev_S = m_motor.getVelocity().getValueAsDouble();
        double velocityRad_S = velocityRev_S * 2 * Math.PI;
        t.log(Level.TRACE, m_name, "velocity (rev_s)", velocityRev_S);
        t.log(Level.DEBUG, m_name, "velocity (rad_s)", velocityRad_S);
        return velocityRad_S;
    }

}
