package org.team100.lib.motor.drive;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance;

import com.revrobotics.CANSparkMax;
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
public class NeoDriveMotor implements Motor100<Distance> {
    private static final int kCurrentLimit = 40;
    private final RelativeEncoder m_encoder;

    private static final double staticFrictionFFVolts = 0.1;

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
    private static final double velocityFFVoltS_Rev = 0.122;

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
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;

    public NeoDriveMotor(
            String name,
            int canId,
            boolean motorPhase,
            double gearRatio,
            double wheelDiameter) {
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(!motorPhase);
        m_motor.setSmartCurrentLimit(kCurrentLimit);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        m_pidController.setPositionPIDWrappingEnabled(false);
        m_pidController.setP(outboardP);
        m_pidController.setI(0);
        m_pidController.setD(0);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-1, 1);

        m_gearRatio = gearRatio;
        m_wheelDiameter = wheelDiameter;

        m_name = String.format("/Neo Turning Motor %s", name);

        t.log(Level.DEBUG, m_name + "/Device ID", m_motor.getDeviceId());
    }

    @Override
    public double get() {
        return m_motor.getAppliedOutput();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.DEBUG, m_name + "/Output", output);
        t.log(Level.DEBUG, m_name + "/Velocity (RPS)", m_encoder.getVelocity() / 60);
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
    public void setVelocity(double outputM_S, double accelM_S2) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_M = motorRev_S * 60;

        double wheelRev_S2 = accelM_S2 / (m_wheelDiameter * Math.PI);
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double velocityFF = velocityFF(motorRev_S);
        double frictionFF = frictionFF(this.getVelocity(), motorRev_S);
        double accelFF = accelFF(motorRev_S2);
        double kFF = frictionFF + velocityFF + accelFF;

        m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage);

        t.log(Level.DEBUG, m_name + "/Output", motorRev_S);
        t.log(Level.DEBUG, m_name + "/Velocity (RPS)", m_encoder.getVelocity() / 60);
    }

    @Override
    public void close() {
        m_motor.close();
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
    private static double velocityFF(double motorRev_S) {
        return velocityFFVoltS_Rev * motorRev_S / saturationVoltage;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S / saturationVoltage;
    }

    public double getVelocity() {
        return m_encoder.getVelocity() / 60;
    }
}
