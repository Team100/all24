package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Phoenix100;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

import com.ctre.phoenix6.controls.VelocityVoltage;

/**
 * PHOENIX 6 VERSION
 * 
 * Swerve drive motor using Falcon 500.
 * 
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * Phoenix 6 uses a Kalman filter to eliminate velocity measurement lag.
 */
public class Falcon6DriveMotor extends Falcon6Motor<Distance100> {

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

    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final double m_distancePerTurn;

    public Falcon6DriveMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            double kDriveReduction,
            double wheelDiameter,
            PIDConstants lowLevelVelocityConstants,
            Feedforward100 ff) {
        super(name, canId, motorPhase, supplyLimit, statorLimit, lowLevelVelocityConstants, ff);
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;
        m_distancePerTurn = wheelDiameter * Math.PI / kDriveReduction;
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S_S) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double wheelRev_S2 = accelM_S_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        double currentMotorRev_S = getVelocityRev_S();

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts;

        VelocityVoltage v = new VelocityVoltage(motorRev_S);
        v.FeedForward = kFFVolts;
        v.Acceleration = motorRev_S2;
        Phoenix100.warn(() -> m_motor.setControl(v));

        t.log(Level.TRACE, m_name, "module input (RPS)", wheelRev_S);
        t.log(Level.TRACE, m_name, "motor input (RPS)", motorRev_S);
        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        log();
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S_S, double torqueNm) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double wheelRev_S2 = accelM_S_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        double currentMotorRev_S = getVelocityRev_S();

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double torqueFFAmps = torqueNm / kTNm_amp;
        double torqueFFVolts = torqueFFAmps * kROhms;

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        VelocityVoltage v = new VelocityVoltage(motorRev_S);
        v.FeedForward = kFFVolts;
        v.Acceleration = motorRev_S2;
        Phoenix100.warn(() -> m_motor.setControl(v));

        t.log(Level.TRACE, m_name, "module input (RPS)", wheelRev_S);
        t.log(Level.TRACE, m_name, "motor input (RPS)", motorRev_S);
        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        log();

    }

    /**
     * @return integrated sensor velocity in rev per sec
     */
    private double getVelocityRev_S() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    /** Position in meters */
    @Override
    public Double getPosition() {
        double positionRev = m_motor.getPosition().getValueAsDouble();
        double positionM = positionRev * m_distancePerTurn;
        t.log(Level.TRACE, m_name, "position (rev)", positionRev);
        t.log(Level.DEBUG, m_name, "position (m)", positionM);

        return positionM;
    }

    /** Velocity in meters/sec */
    @Override
    public double getRate() {
        double velocityRev_S = m_motor.getVelocity().getValueAsDouble();
        double velocityM_S = velocityRev_S * m_distancePerTurn;
        t.log(Level.TRACE, m_name, "velocity (rev_s)", velocityRev_S);
        t.log(Level.DEBUG, m_name, "velocity (m_s)", velocityM_S);

        return velocityM_S;
    }

}
