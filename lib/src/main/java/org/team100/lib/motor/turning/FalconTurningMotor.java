package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

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
 * Swerve steering motor using Falcon 500.
 * 
 * See {@link FalconDriveMotor} for configuration details.
 */
public class FalconTurningMotor implements Motor100<Angle> {
    private static final double ticksPerRevolution = 2048;
    private static final double gearRatio = 10.29;
    private static final double kCurrentLimit = 40;

    /**
     * The speed, below which, static friction applies, in motor revolutions per
     * second.
     */
    private static final double staticFrictionSpeedLimitRev_S = 1.0;

    /**
     * Friction feedforward in volts, for when the mechanism is stopped, or nearly
     * so.
     */
    private static final double staticFrictionFFVolts = 0.375;

    /**
     * Friction feedforward in volts, for when the mechanism is moving.
     */
    private static final double dynamicFrictionFFVolts = 0.27;

    /**
     * Velocity feedforward in units of volts per motor revolution per second, or
     * volt-seconds per revolution. Since saturation is 11 volts and free speed is
     * about 100 rev/s, this is about 0.11.
     */
    private static final double velocityFFVoltS_Rev = 0.11;

    /**
     * Placeholder for accel feedforward.
     */
    private static final double accelFFVoltS2_Rad = 0;

    /**
     * Proportional feedback coefficient for the controller. The error is measured
     * in sensor units (ticks per 100ms), and the full scale output is 1023.
     */
    private static final double outboardP = 0.1;

    /**
     * For voltage compensation, the maximum output voltage.
     */
    private static final double saturationVoltage = 11;

    private final Telemetry t = Telemetry.get();
    private final TalonFX m_motor;
    private final String m_name;

    public FalconTurningMotor(String name, int canId) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();

        m_motor = new TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.setNeutralMode(NeutralMode.Brake);

        // the serve module steering gear is inverted
        m_motor.setInverted(InvertType.InvertMotorOutput);

        // configure current limits
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));

        // use integrated sensor for status and PID feedback
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

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

        m_name = String.format("/%s/Falcon Turning Motor", name);
        t.log(Level.DEBUG, m_name + "/Device ID", m_motor.getDeviceID());
    }

    @Override
    public double get() {
        return m_motor.getMotorOutputPercent();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(ControlMode.PercentOutput, output);

        t.log(Level.DEBUG, m_name + "/output [-1,1]", output);
    }

    /**
     * Supports accel feedforward.
     */
    public void setVelocity(double outputRad_S, double accelRad_S_S) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * gearRatio;
        double motorRev_100ms = motorRev_S / 10;
        double motorTick_100ms = motorRev_100ms * ticksPerRevolution;

        double currentMotorRev_S = currentMotorRev_S();
        double frictionFF = frictionFF(currentMotorRev_S, motorRev_S);
        double velocityFF = velocityFF(motorRev_S);
        double accelFF = accelFF(accelRad_S_S);
        double kFF = frictionFF + velocityFF + accelFF;

        m_motor.set(ControlMode.Velocity, motorTick_100ms, DemandType.ArbitraryFeedForward, kFF);

        t.log(Level.DEBUG, m_name + "/error rev_s", getErrorRev_S());
        t.log(Level.DEBUG, m_name + "/current speed rev_s", currentMotorRev_S);
        t.log(Level.DEBUG, m_name + "/output [-1, 1]", get());
    }

    @Override
    public void stop() {
        m_motor.neutralOutput();
    }

    @Override
    public void close() {
        m_motor.DestroyObject();
    }

    //////////////////////////////////////////////////////////////////

    /** Velocity feedforward in duty cycle units [-1, 1] */
    private static double velocityFF(double desiredMotorRev_S) {
        return velocityFFVoltS_Rev * desiredMotorRev_S / saturationVoltage;
    }

    /** Frictional feedforward in duty cycle units [-1, 1] */
    private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < staticFrictionSpeedLimitRev_S) {
            return staticFrictionFFVolts * direction / saturationVoltage;
        }
        return dynamicFrictionFFVolts * direction / saturationVoltage;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelRad_S_S) {
        return accelFFVoltS2_Rad * accelRad_S_S / saturationVoltage;
    }

    /**
     * Current speed in revolutions per second. Note: this measurement is delayed
     * and filtered.
     */
    private double currentMotorRev_S() {
        double motorTick_100ms = m_motor.getSelectedSensorVelocity();
        double motorRev_100ms = motorTick_100ms / ticksPerRevolution;
        return motorRev_100ms * 10;
    }

    private double getErrorRev_S() {
        double errorTick_100ms = m_motor.getClosedLoopError();
        double errorRev_100ms = errorTick_100ms / ticksPerRevolution;
        return errorRev_100ms * 10;
    }

}
