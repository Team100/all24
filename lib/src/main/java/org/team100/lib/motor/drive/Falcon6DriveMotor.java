package org.team100.lib.motor.drive;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.Phoenix100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * PHOENIX 6 VERSION
 * 
 * Swerve drive motor using Falcon 500.
 * 
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * Phoenix 6 uses a Kalman filter to eliminate velocity measurement lag.
 */
public class Falcon6DriveMotor implements MotorWithEncoder100<Distance100> {

    /**
     * Motor resistance, Kraken. Falcon is 0.03.
     * https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
     */
    private static final double kROhms = 0.025;
    /**
     * Motor torque constant, Kraken. Falcon is 0.018.
     * https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
     */
    private static final double kTNm_amp = 0.019;

    private final Feedforward100 m_ff;

    private final Telemetry t = Telemetry.get();
    private final TalonFX m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;
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
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_ff = ff;
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;
        m_distancePerTurn = wheelDiameter * Math.PI / kDriveReduction;
        m_motor = new TalonFX(canId);

        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();
        Phoenix100.baseConfig(talonFXConfigurator);
        Phoenix100.motorConfig(talonFXConfigurator, motorPhase);
        Phoenix100.currentConfig(talonFXConfigurator, supplyLimit, statorLimit);
        Phoenix100.pidConfig(talonFXConfigurator, lowLevelVelocityConstants);
        Phoenix100.crash(() -> m_motor.getVelocity().setUpdateFrequency(50));

        m_name = Names.append(name, this);
        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceID());
    }

    //////////////////
    // motor methods

    @Override
    public void setDutyCycle(double output) {
        DutyCycleOut d = new DutyCycleOut(output);
        Phoenix100.warn(() -> m_motor.setControl(d));
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
    }

    /**
     * Supports accel feedforward.
     */
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

        t.log(Level.DEBUG, m_name, "output [-1,1]", m_motor.getDutyCycle().getValueAsDouble());
        t.log(Level.TRACE, m_name, "error (rev_s)", m_motor.getClosedLoopError().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getDeviceTemp().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getSupplyCurrent().getValueAsDouble());

    }

    /**
     * Supports accel feedforward.
     */
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

        t.log(Level.DEBUG, m_name, "output [-1,1]", m_motor.getDutyCycle().getValueAsDouble());
        t.log(Level.TRACE, m_name, "error (rev_s)", m_motor.getClosedLoopError().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getDeviceTemp().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getSupplyCurrent().getValueAsDouble());

    }

    @Override
    public double getTorque() {
        StatusSignal<Double> statorCurrentAmpsStatus = m_motor.getTorqueCurrent();
        // TODO: latency compensation
        double statorCurrentAmps = statorCurrentAmpsStatus.getValueAsDouble();
        return statorCurrentAmps * kTNm_amp;
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * @return integrated sensor velocity in rev per sec
     */
    public double getVelocityRev_S() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        m_motor.setPosition(0);
    }

    //////////////////////////
    // encoder methods

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

    @Override
    public void reset() {
        resetPosition();
    }

    ///////////////////////////////////////////////////////////////

}
