package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;

/**
 * Swerve steering motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoTurningMotor implements Motor100<Angle100> {
    private final RelativeEncoder m_encoder;

    private static final double staticFrictionFFVolts = 0.1;
    /**
     * This is surely wrong.
     */
    private final double m_gearRatio;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private static final double dynamicFrictionFFVolts = 0.065;

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
    private static final double accelFFVoltS2_M = 0;

    /**
     * Proportional feedback coefficient for the controller.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private static final double outboardP = 0.0001;

    /**
     * For voltage compensation, the maximum output voltage.
     */
    private static final double saturationVoltage = 1;

    private final Telemetry t = Telemetry.get();
    private final SparkPIDController m_pidController;
    private final CANSparkMax m_motor;
    private final String m_name;

    /** Current velocity measurement, obtained in periodic(). */
    private double m_encoderVelocity;
    /** Current position measurement, obtained in periodic(). */
    private double m_encoderPosition;

    public NeoTurningMotor(String name, int canId, boolean motorPhase, int currentLimit, double gearRatio, double kV, PIDController lowLevelVelocityController) {
        velocityFFVoltS_Rev = kV;
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        require(m_motor.restoreFactoryDefaults());
        m_gearRatio = gearRatio;
        m_motor.setInverted(!motorPhase);
        require(m_motor.setSmartCurrentLimit(currentLimit));

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        require(m_pidController.setPositionPIDWrappingEnabled(true));
        require(m_pidController.setP(lowLevelVelocityController.getP()));
        require(m_pidController.setI(lowLevelVelocityController.getI()));
        require(m_pidController.setD(lowLevelVelocityController.getD()));
        require(m_pidController.setIZone(lowLevelVelocityController.getIZone()));
        require(m_pidController.setFF(0));
        require(m_pidController.setOutputRange(-1, 1));

        m_name = Names.append(name, this);

        t.log(Level.DEBUG, m_name, "Device ID", m_motor.getDeviceId());
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
        t.log(Level.DEBUG, m_name, "desired duty cycle [-1,1]", output);
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

        t.log(Level.DEBUG, m_name, "friction feedforward [-1,1]", frictionFF);
        t.log(Level.DEBUG, m_name, "velocity feedforward [-1,1]", velocityFF);
        t.log(Level.DEBUG, m_name, "accel feedforward [-1,1]", accelFF);
        t.log(Level.DEBUG, m_name, "desired speed (rev_s)", motorRevs_S);
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
        t.log(Level.DEBUG, m_name, "position (rev)", m_encoderPosition);
        t.log(Level.DEBUG, m_name, "velocity (rev_s)", m_encoderVelocity / 60);
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.DEBUG, m_name, "duty cycle", m_motor.getAppliedOutput());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getMotorTemperature());
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
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
        return velocityFFVoltS_Rev * motorRev_S / saturationVoltage;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S / saturationVoltage;
    }
}
