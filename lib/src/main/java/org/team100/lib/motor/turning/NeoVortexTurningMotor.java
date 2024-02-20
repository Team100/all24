package org.team100.lib.motor.turning;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

/**
 * Swerve steering motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoVortexTurningMotor implements Motor100<Angle100> {
    private final RelativeEncoder m_encoder;

    private final double staticFrictionFFVolts;
    /**
     * This is surely wrong.
     */
    private final double m_gearRatio;

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

    /**
     * Proportional feedback coefficient for the controller.
     * 
     * This is a guess. Calibrate it before using it.
     */

    private final Telemetry t = Telemetry.get();
    private final SparkPIDController m_pidController;
    private final CANSparkFlex m_motor;
    private final String m_name;

    /** Current velocity measurement, obtained in periodic(). */
    private double m_encoderVelocity;
    /** Current position measurement, obtained in periodic(). */
    private double m_encoderPosition;

    public NeoVortexTurningMotor(String name,
            int canId,
            MotorPhase motorPhase,
            int currentLimit,
            double gearRatio,
            FeedforwardConstants lowLevelFeedforwardConstants,
            PIDConstants lowLevelVelocityConstants) {
        velocityFFVoltS_Rev = lowLevelFeedforwardConstants.getkV();
        accelFFVoltS2_M = lowLevelFeedforwardConstants.getkA();
        dynamicFrictionFFVolts = lowLevelFeedforwardConstants.getkDS();
        staticFrictionFFVolts = lowLevelFeedforwardConstants.getkSS();
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        require(m_motor.restoreFactoryDefaults());
        m_gearRatio = gearRatio;

        if(motorPhase == MotorPhase.FORWARD){
            m_motor.setInverted(false);
        } else {
            m_motor.setInverted(true);
        }

        require(m_motor.setSmartCurrentLimit(currentLimit));

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        require(m_pidController.setPositionPIDWrappingEnabled(true));
        
        setP(lowLevelVelocityConstants.getP());
        setI(lowLevelVelocityConstants.getI());
        setD(lowLevelVelocityConstants.getD());
        setIZone(lowLevelVelocityConstants.getIZone());
        
        require(m_pidController.setFF(0));
        require(m_pidController.setOutputRange(-1, 1));

        m_name = Names.append(name, this);

        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceId());

        t.register(Level.TRACE, m_name, "P", lowLevelVelocityConstants.getP(), this::setP);
        t.register(Level.TRACE, m_name, "I", lowLevelVelocityConstants.getI(), this::setI);
        t.register(Level.TRACE, m_name, "D", lowLevelVelocityConstants.getD(), this::setD);
        t.register(Level.TRACE, m_name, "IZone", lowLevelVelocityConstants.getIZone(), this::setIZone);
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
        // TODO: make this throw
        if (responseCode != REVLibError.kOk)
            Util.warn("NeoTurningMotor received response code " + responseCode.name());
        // throw new IllegalStateException();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Supports accel feedforward.
     * 
     * Note the implementation here is surely wrong, it needs to be calibrated.
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        double motorRad_S = m_gearRatio * outputRad_S;
        double motorRevs_S = motorRad_S / (2 * Math.PI);
        double motorRevs_M = motorRevs_S * 60;
        double motorRad_S2 = m_gearRatio * accelRad_S2;
        double motorRevs_S2 = motorRad_S2 / (2 * Math.PI);
        double velocityFF = velocityFF(motorRevs_S);
        double frictionFF = frictionFF(m_encoderVelocity / 60, motorRevs_S);
        double accelFF = accelFF(motorRevs_S2);
        double kFF = frictionFF + velocityFF + accelFF;

        m_pidController.setReference(motorRevs_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage);

        t.log(Level.TRACE, m_name, "friction feedforward [-1,1]", frictionFF);
        t.log(Level.TRACE, m_name, "velocity feedforward [-1,1]", velocityFF);
        t.log(Level.TRACE, m_name, "accel feedforward [-1,1]", accelFF);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRevs_S);
    }

    public void resetPosition() {
        m_encoder.setPosition(0);
        m_encoderPosition = 0;
    }

    @Override
    public void close() {
        m_motor.close();
    }

    public double getPositionRot() {
        return m_encoderPosition;
    }

    public double getRateRPM() {
        return m_encoderVelocity;
    }

    /**
     * Update measurements.
     */
    public void periodic() {
        m_encoderPosition = m_encoder.getPosition();
        m_encoderVelocity = m_encoder.getVelocity();
        t.log(Level.TRACE, m_name, "position (rev)", m_encoderPosition);
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_encoderVelocity / 60);
        t.log(Level.TRACE, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.TRACE, m_name, "duty cycle", m_motor.getAppliedOutput());
        t.log(Level.TRACE, m_name, "temperature (C)", m_motor.getMotorTemperature());
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < 0.5) {
            return staticFrictionFFVolts * direction;
        }
        return dynamicFrictionFFVolts * direction;
    }

    /**
     * Velocity feedforward in duty cycle units [-1, 1]
     */
    private double velocityFF(double motorRev_S) {
        return velocityFFVoltS_Rev * motorRev_S;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S;
    }
}