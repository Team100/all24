package org.team100.lib.motor.drive;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * Linear drive motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoVortexDriveMotor implements Motor100<Distance100> {
    /**
     * Motor resistance https://www.revrobotics.com/rev-21-1652/
     */
    private static final double kROhms = 0.057;
    /**
     * Motor torque constant https://www.revrobotics.com/rev-21-1652/
     */
    private static final double kTNm_amp = 0.017;

    private final RelativeEncoder m_encoder;

    private final double staticFrictionFFVolts;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private final double dynamicFrictionFFVolts;

    /**
     * Velocity feedforward in units of volts per motor revolution per second, or
     * volt-seconds per revolution.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private final double velocityFFVoltS_Rev;

    /**
     * Placeholder for accel feedforward.
     */
    private final double accelFFVoltS2_M;
    private final Telemetry t = Telemetry.get();
    private final SparkPIDController m_pidController;
    private final CANSparkFlex m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;

    /**
     * 
     * @param name
     * @param canId
     * @param motorPhase
     * @param currentLimit
     * @param gearRatio
     * @param wheelDiameter
     * @param lowLevelFeedforwardConstants using VOLTS VOLTS VOLTS
     * @param lowLevelVelocityConstants
     */
    public NeoVortexDriveMotor(
            String name,
            int canId,
            boolean motorPhase,
            int currentLimit,
            double gearRatio,
            double wheelDiameter,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        require(m_motor.restoreFactoryDefaults());
        accelFFVoltS2_M = lowLevelFeedforwardConstants.getkA();
        velocityFFVoltS_Rev = lowLevelFeedforwardConstants.getkV();
        staticFrictionFFVolts = lowLevelFeedforwardConstants.getkSS();
        dynamicFrictionFFVolts = lowLevelFeedforwardConstants.getkDS();
        m_motor.setInverted(!motorPhase);
        require(m_motor.setSmartCurrentLimit(currentLimit));
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        require(m_pidController.setPositionPIDWrappingEnabled(false));
        m_pidController.setP(lowLevelVelocityConstants.getP());
        m_pidController.setI(lowLevelVelocityConstants.getI());
        m_pidController.setD(lowLevelVelocityConstants.getD());
        m_pidController.setIZone(lowLevelVelocityConstants.getIZone());
        require(m_pidController.setFF(0));
        require(m_pidController.setOutputRange(-1, 1));
        m_motor.setIdleMode(IdleMode.kBrake);
        m_gearRatio = gearRatio;
        m_wheelDiameter = wheelDiameter;

        m_name = Names.append(name, this);

        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceId());
        t.register(Level.TRACE, m_name, "P", lowLevelVelocityConstants.getP(), this::setP);
        t.register(Level.TRACE, m_name, "I", lowLevelVelocityConstants.getI(), this::setI);
        t.register(Level.TRACE, m_name, "D", lowLevelVelocityConstants.getD(), this::setD);
        t.register(Level.TRACE, m_name, "IZone", lowLevelVelocityConstants.getIZone(), this::setIZone);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "Output", output);
        log();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Using the supplied wheel diameter and gear ratio, set the motor velocity
     * to the correct RPM given the desired linear speed in m/s.
     * 
     * Supports accel feedforward.
     * 
     * Note the implementation here is surely wrong, it needs to be calibrated.
     */
    @Override
    public void setVelocity(double outputM_S, double accelM_S2) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_M = motorRev_S * 60;

        double wheelRev_S2 = accelM_S2 / (m_wheelDiameter * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double velocityFFVolts = velocityFFVolts(motorRev_S);
        double frictionFFVolts = frictionFFVolts(m_encoder.getVelocity() / 60, motorRev_S);
        double accelFFVolts = accelFFVolts(motorRev_S2);
        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts;

        m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage);

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRev_S);
        log();
    }

    @Override
    public void setVelocity(double outputM_S, double accelM_S2, double torqueNm) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_M = motorRev_S * 60;

        double wheelRev_S2 = accelM_S2 / (m_wheelDiameter * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double velocityFFVolts = velocityFFVolts(motorRev_S);
        double frictionFFVolts = frictionFFVolts(m_encoder.getVelocity() / 60, motorRev_S);
        double accelFFVolts = accelFFVolts(motorRev_S2);

        double torqueFFAmps = torqueNm / kTNm_amp;
        double torqueFFVolts = torqueFFAmps * kROhms;

        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage);

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRev_S);
        log();
    }

    @Override
    public double getTorque() {
        return m_motor.getOutputCurrent() * kTNm_amp;
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * @return integrated sensor position in rotations.
     */
    public double getPositionRot() {
        return m_encoder.getPosition();
    }

    /**
     * @return integrated sensor velocity in RPM
     */
    public double getRateRPM() {
        return m_encoder.getVelocity();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        m_encoder.setPosition(0);
    }

    public void log() {
        t.log(Level.TRACE, m_name, "position (rev)", m_encoder.getPosition());
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_encoder.getVelocity() / 60);
        t.log(Level.TRACE, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.TRACE, m_name, "duty cycle", m_motor.getAppliedOutput());
        t.log(Level.TRACE, m_name, "temperature (C)", m_motor.getMotorTemperature());
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in VOLTS
     */
    private double frictionFFVolts(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < 0.5) {
            return staticFrictionFFVolts * direction;
        }
        return dynamicFrictionFFVolts * direction;
    }

    /**
     * Velocity feedforward in VOLTS
     */
    private double velocityFFVolts(double motorRev_S) {
        return velocityFFVoltS_Rev * motorRev_S;
    }

    /**
     * Acceleration feedforward VOLTS
     */
    private double accelFFVolts(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S;
    }

    private void setP(double p) {
        m_pidController.setP(p);
    }

    private void setI(double i) {
        m_pidController.setI(i);
    }

    private void setD(double d) {
        m_pidController.setD(d);
    }

    private void setIZone(double iz) {
        m_pidController.setIZone(iz);
    }

    private void require(REVLibError responseCode) {
        if (responseCode != REVLibError.kOk)
            throw new IllegalStateException("NeoDriveMotor received response code " + responseCode.name());
    }

}
