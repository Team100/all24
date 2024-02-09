package org.team100.lib.motor.drive;

import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
     */
    private static final double outboardP = .001;

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
    private final double m_distancePerTurn;

    /** Current position, updated in periodic(). */
    private double m_rawPosition;
    /** Current velocity, updated in periodic(). */
    private double m_velocityRev_S;
    /** Current output, updated in periodic() */
    private double m_output;
    /** Current motor error, updated in periodic() */
    private double m_error;

    /** updated in periodic() */
    private double m_positionM;
    /** updated in periodic() */
    private double m_velocityM_S;

    public Falcon6DriveMotor(
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
        m_distancePerTurn = wheelDiameter * Math.PI / kDriveReduction;

        m_motor = new TalonFX(canId);

        // m_motor.configFactoryDefault();
        var talonFXConfigurator = m_motor.getConfigurator();

        TalonFXConfiguration conf = new TalonFXConfiguration();
        talonFXConfigurator.apply(conf);

        // m_motor.setNeutralMode(NeutralMode.Brake);

        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentConfigs = new CurrentLimitsConfigs();
        // i think maybe we don't care about this?
        // currentConfigs.StatorCurrentLimit = kCurrentLimit;
        // currentConfigs.StatorCurrentLimitEnable = true;
        // we're just trying to avoid draining the battery too much.
        currentConfigs.SupplyCurrentLimit = currentLimit;
        currentConfigs.SupplyCurrentLimitEnable = true;
        talonFXConfigurator.apply(currentConfigs);

        // // configure current limits
        // m_motor.configStatorCurrentLimit(
        // new StatorCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));
        // m_motor.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));

        // use integrated sensor for status and PID feedback
        // m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        if (motorPhase) {
            motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            // m_motor.setInverted(InvertType.None);
        } else {
            motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
            // m_motor.setInverted(InvertType.InvertMotorOutput);
        }

        talonFXConfigurator.apply(motorConfigs);

        // configure velocity measurement sampling
        // m_motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);
        // m_motor.configVelocityMeasurementWindow(8);

        // configure CAN bus velocity measurement reporting period
        // m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        m_motor.getVelocity().setUpdateFrequency(50);

        // configure voltage compensation
        // m_motor.configVoltageCompSaturation(saturationVoltage);
        // m_motor.enableVoltageCompensation(true);

        // configure output limits
        // m_motor.configNominalOutputForward(0);
        // m_motor.configNominalOutputReverse(0);
        // m_motor.configPeakOutputForward(1);
        // m_motor.configPeakOutputReverse(-1);

        // configure outboard PID
        // m_motor.config_kP(0, outboardP);
        // m_motor.config_kI(0, 0);
        // m_motor.config_kD(0, 0);
        // m_motor.config_kF(0, 0);

        // set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.0;
        slot0Configs.kP = outboardP;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        // apply gains, 50 ms total timeout
        m_motor.getConfigurator().apply(slot0Configs, 0.050);

        m_name = Names.append(name, this);
        t.log(Level.DEBUG, m_name, "Device ID", m_motor.getDeviceID());
    }

    //////////////////
    // motor methods

    @Override
    public void setDutyCycle(double output) {
        DutyCycleOut d = new DutyCycleOut(output);
        m_motor.setControl(d);
        // m_motor.set(ControlMode.PercentOutput, output);
        t.log(Level.DEBUG, m_name, "desired duty cycle [-1,1]", output);
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
        // double motorRev_100ms = motorRev_S / 10;
        // double motorTick_100ms = motorRev_100ms * ticksPerRevolution;

        double currentMotorRev_S = m_velocityRev_S;
        double frictionFF = frictionFF(currentMotorRev_S, motorRev_S);
        double velocityFF = velocityFF(motorRev_S);
        double accelFF = accelFF(accelM_S_S);
        double kFF = frictionFF + velocityFF + accelFF;

        VelocityDutyCycle v = new VelocityDutyCycle(motorRev_S);
        v.FeedForward = kFF;
        v.EnableFOC = true;
        v.Acceleration = motorRev_S2;
        m_motor.setControl(v);

        // m_motor.set(ControlMode.Velocity, motorTick_100ms,
        // DemandType.ArbitraryFeedForward, kFF);
        t.log(Level.DEBUG, m_name, "motor input", motorRev_S);
        t.log(Level.DEBUG, m_name, "friction feedforward [-1,1]", frictionFF);
        t.log(Level.DEBUG, m_name, "velocity feedforward [-1,1]", velocityFF);
        t.log(Level.DEBUG, m_name, "accel feedforward [-1,1]", accelFF);
        // t.log(Level.DEBUG, m_name, "desired speed 2048ths_100ms", motorTick_100ms);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
        // m_motor.neutralOutput();
    }

    @Override
    public void close() {
        // m_motor.DestroyObject();
        m_motor.close();
    }

    // /**
    // * @return integrated sensor position in sensor units (1/2048 turn).
    // */
    // public double getPosition() {
    // return m_rawPosition;
    // }

    /**
     * @return integrated sensor velocity in rev per sec
     */
    public double getVelocityRev_S() {
        return m_velocityRev_S;
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        // m_motor.setSelectedSensorPosition(0);
        m_motor.setPosition(0);
        m_rawPosition = 0;
    }

    @Override
    public void periodic() {
        // m_rawPosition = m_motor.getSelectedSensorPosition();
        m_rawPosition = m_motor.getPosition().getValueAsDouble();
        m_velocityRev_S = m_motor.getVelocity().getValueAsDouble();

        // m_output = m_motor.getMotorOutputPercent();
        m_output = m_motor.getDutyCycle().getValueAsDouble();
        m_error = m_motor.getClosedLoopError().getValueAsDouble();

        m_positionM = m_rawPosition * m_distancePerTurn;
        m_velocityM_S = m_velocityRev_S * m_distancePerTurn;

        t.log(Level.DEBUG, m_name, "position (rev)", m_rawPosition);
        t.log(Level.DEBUG, m_name, "position (m)", m_positionM);
        t.log(Level.DEBUG, m_name, "velocity (rev_s)", m_velocityRev_S);
        t.log(Level.DEBUG, m_name, "velocity (m_s)", m_velocityM_S);

        t.log(Level.DEBUG, m_name, "output [-1,1]", m_output);
        t.log(Level.DEBUG, m_name, "error (rev_s)", getErrorRev_S());
        // t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getTemperature());
        t.log(Level.DEBUG, m_name, "temperature (C)", m_motor.getDeviceTemp().getValueAsDouble());
        t.log(Level.DEBUG, m_name, "current (A)", m_motor.getSupplyCurrent().getValueAsDouble());
        // t.log(Level.DEBUG, m_name, "last error code", m_motor.getLastError());
    }

    //////////////////////////
    // encoder methods

    /** Position in meters */
    @Override
    public double getPosition() {
        return m_positionM;
    }

    /** Velocity in meters/sec */
    @Override
    public double getRate() {
        return m_velocityM_S;
    }

    @Override
    public void reset() {
        resetPosition();
        m_positionM = 0;
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Frictional feedforward in duty cycle units [-1, 1]
     */
    private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < staticFrictionSpeedLimitRev_S) {
            return staticFrictionFFVolts * direction;
        }
        return dynamicFrictionFFVolts * direction;
    }

    /**
     * Velocity feedforward in duty cycle units [-1, 1]
     */
    private static double velocityFF(double desiredMotorRev_S) {
        return velocityFFVoltS_Rev * desiredMotorRev_S;
    }

    /**
     * Acceleration feedforward in duty cycle units [-1, 1]
     */
    private static double accelFF(double accelM_S_S) {
        return accelFFVoltS2_M * accelM_S_S;
    }

    private double getErrorRev_S() {
        double errorTick_100ms = m_error;
        double errorRev_100ms = errorTick_100ms;
        return errorRev_100ms * 10;
    }
}
