package org.team100.lib.motor.drive;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Swerve drive motor using Falcon 500.
 * 
 * Uses default position/velocity sensor which is the integrated one.
 * 
 * See details on velocity averaging and sampling.
 * https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#velocity-measurement-filter
 * 
 * Summarized:
 * 
 * * every 1ms, the velocity is measured as (x(t) - x(t-p)) / p
 * * N such measurements are averaged
 * 
 * Default p is 100 ms, default N is 64, so, if the velocity measurement is
 * perfect, it represents the actual velocity 82 ms ago, which is a long time.
 * 
 * A better setting would use a window that's about the width of the control
 * period, 20ms, so try 10ms and 8 samples, so the measurement will represent
 * the velocity about 10 ms ago.
 * 
 * If we're aiming for a quantization error of 10%, we need 10 steps between
 * position samples, or 10/2048ths of a revolution, and using a period of 10 ms,
 * the minimum rotational rate would be about 0.5 rev/s.
 * 
 * For onboard (roboRIO) feedback, there is additional delay in the velocity
 * measurement caused by the periodicity of the CAN status report, which can be
 * adjusted. The default is 20 ms; reducing it too much floods the CAN bus, so
 * we chose 10 ms below, which is a common value.
 * 
 * Details on CAN bus status frame timing:
 * https://v5.docs.ctr-electronics.com/en/latest/ch18_CommonAPI.html#setting-status-frame-periods
 */
public class FalconDriveMotor implements Motor100<Distance100> {

    /**
     * The speed, below which, static friction applies, in motor revolutions per
     * second.
     */
    private static final double staticFrictionSpeedLimitRev_S = 3.5;

    /**
     * Friction feedforward in volts, for when the mechanism is stopped, or nearly
     * so.
     */
    private static final double staticFrictionFFVolts = 0.18;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     * 
     * This value seems very low, perhaps because the falcon closed-loop control is
     * compensating?
     */
    private static final double dynamicFrictionFFVolts = 0.01;

    /**
     * Velocity feedforward in units of volts per motor revolution per second, or
     * volt-seconds per revolution. Since saturation is 11 volts and free speed is
     * about 100 rev/s, this is about 0.11.
     */
    private static final double velocityFFVoltS_Rev = 0.11;

    /**
     * Placeholder for accel feedforward.
     */
    private static final double accelFFVoltS2_M = 0;

    /**
     * Proportional feedback coefficient for the controller. The error is measured
     * in sensor units (ticks per 100ms), and the full scale output is 1023.
     */
    private static final double outboardP = 0.05;

    /**
     * The Falcon 500 onboard sensor.
     */
    private static final double ticksPerRevolution = 2048;

    /**
     * For voltage compensation, the maximum output voltage.
     */
    private static final double saturationVoltage = 11;

    private final Telemetry t = Telemetry.get();
    private final TalonFX m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;
    private final String m_name;

    /** Current position, updated in periodic(). */
    private double m_rawPosition;
    /** Current velocity, updated in periodic(). */
    private double m_rawVelocity;
    /** Current output, updated in periodic() */
    private double m_output;
    /** Current motor error, updated in periodic() */
    private double m_error;

    public FalconDriveMotor(
            String name,
            int canId,
            boolean motorPhase,
            double currentLimit,
            double kDriveReduction,
            double wheelDiameter) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;

        m_motor = new TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);

        // configure current limits
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));

        // use integrated sensor for status and PID feedback
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        if (motorPhase) {
            m_motor.setInverted(InvertType.None);
        } else {
            m_motor.setInverted(InvertType.InvertMotorOutput);
        }
        // configure velocity measurement sampling
        m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        m_motor.configVelocityMeasurementWindow(8);

        // configure CAN bus velocity measurement reporting period
        m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        // configure voltage compensation
        m_motor.configVoltageCompSaturation(saturationVoltage);
        m_motor.enableVoltageCompensation(true);

        // configure output limits
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);

        // configure outboard PID
        m_motor.config_kP(0, outboardP);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
        m_motor.config_kF(0, 0);

        m_name = Names.append(name, this);
        t.log(Level.DEBUG, m_name, "Device ID", m_motor.getDeviceID());
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
        t.log(Level.DEBUG, m_name, "desired duty cycle [-1,1]", output);
    }

    /**
     * Supports accel feedforward.
     */
    @Override
    public void setVelocity(double outputM_S, double accelM_S_S) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorRev_100ms = motorRev_S / 10;
        double motorTick_100ms = motorRev_100ms * ticksPerRevolution;

        double currentMotorRev_S = currentMotorRev_S();
        double frictionFF = frictionFF(currentMotorRev_S, motorRev_S);
        double velocityFF = velocityFF(motorRev_S);
        double accelFF = accelFF(accelM_S_S);
        double kFF = frictionFF + velocityFF + accelFF;

        m_motor.set(ControlMode.Velocity, motorTick_100ms, DemandType.ArbitraryFeedForward, kFF);

        t.log(Level.DEBUG, m_name, "friction feedforward [-1,1]", frictionFF);
        t.log(Level.DEBUG, m_name, "velocity feedforward [-1,1]", velocityFF);
        t.log(Level.DEBUG, m_name, "accel feedforward [-1,1]", accelFF);
        t.log(Level.DEBUG, m_name, "desired speed 2048ths_100ms", motorTick_100ms);
    }

    @Override
    public void stop() {
        m_motor.neutralOutput();
    }

    @Override
    public void close() {
        m_motor.DestroyObject();
    }

    /**
     * @return integrated sensor position in sensor units (1/2048 turn).
     */
    public double getPosition() {
        return m_rawPosition;
    }

    /**
     * @return integrated sensor velocity in sensor units (1/2048 turn) per 100ms.
     */
    public double getVelocity2048_100() {
        return m_rawVelocity;
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        m_motor.setSelectedSensorPosition(0);
        m_rawPosition = 0;
    }

    @Override
    public void periodic() {
        m_rawPosition = m_motor.getSelectedSensorPosition();
        m_rawVelocity = m_motor.getSelectedSensorVelocity();
        m_output = m_motor.getMotorOutputPercent();
        m_error = m_motor.getClosedLoopError();
        t.log(Level.DEBUG, m_name, "position (raw)", m_rawPosition);
        t.log(Level.DEBUG, m_name, "velocity (raw)", m_rawVelocity);
        t.log(Level.DEBUG, m_name, "velocity (rev_s)", currentMotorRev_S());
        t.log(Level.DEBUG, m_name, "output [-1,1]", m_output);
        t.log(Level.DEBUG, m_name, "error (rev_s)", getErrorRev_S());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getTemperature());
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getSupplyCurrent());        
        t.log(Level.DEBUG, m_name, "last error code", m_motor.getLastError());        
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < staticFrictionSpeedLimitRev_S) {
            return staticFrictionFFVolts * direction / saturationVoltage;
        }
        return dynamicFrictionFFVolts * direction / saturationVoltage;
    }

    /**
     * Velocity feedforward in duty cycle units [-1, 1]
     */
    private static double velocityFF(double desiredMotorRev_S) {
        return velocityFFVoltS_Rev * desiredMotorRev_S / saturationVoltage;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S / saturationVoltage;
    }

    /**
     * Current speed in revolutions per second. Note: this measurement is delayed
     * and filtered.
     */
    private double currentMotorRev_S() {
        double motorTick_100ms = m_rawVelocity;
        double motorRev_100ms = motorTick_100ms / ticksPerRevolution;
        return motorRev_100ms * 10;
    }

    private double getErrorRev_S() {
        double errorTick_100ms = m_error;
        double errorRev_100ms = errorTick_100ms / ticksPerRevolution;
        return errorRev_100ms * 10;
    }
}
