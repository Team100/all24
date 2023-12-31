package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

/**
 * Swerve steering motor using REV Neo.
 * 
 * This is not finished, don't use it without finishing it.
 */
public class NeoTurningMotor implements Motor100<Angle> {
    private static final int kCurrentLimit = 40;
    /**
     * This is surely wrong.
     */
    private static final double kMotorGearing = 1;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private static final double dynamicFrictionFFVolts = 0.1;

    /**
     * Velocity feedforward in units of volts per motor revolution per second, or
     * volt-seconds per revolution.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private static final double velocityFFVoltS_Rev = 0.11;

    /**
     * Placeholder for accel feedforward.
     */
    private static final double accelFFVoltS2_M = 0;

    /**
     * Proportional feedback coefficient for the controller.
     * 
     * This is a guess. Calibrate it before using it.
     */
    private static final double outboardP = 0.1;

    /**
     * For voltage compensation, the maximum output voltage.
     */
    private static final double saturationVoltage = 11;

    private final Telemetry t = Telemetry.get();
    private final SparkMaxPIDController m_pidController;
    private final CANSparkMax m_motor;
    private final String m_name;

    public NeoTurningMotor(String name, int canId) {
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(kCurrentLimit);

        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

        m_pidController = m_motor.getPIDController();
        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setP(outboardP);
        m_pidController.setI(0);
        m_pidController.setD(0);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-1, 1);

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
        double motorRad_S = kMotorGearing * outputRad_S;

        double velocityFF = velocityFF(motorRad_S);
        double frictionFF = frictionFF(motorRad_S);
        double accelFF = accelFF(accelRad_S2);
        double kFF = frictionFF + velocityFF + accelFF;

        m_pidController.setReference(motorRad_S, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage);

        t.log(Level.DEBUG, m_name + "/Output", outputRad_S);
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /////////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private static double frictionFF(double motorRev_S) {
        return dynamicFrictionFFVolts * Math.signum(motorRev_S) / saturationVoltage;
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
}
