package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * PHOENIX 6 VERSION
 * 
 * Swerve steering motor using Falcon 500.
 * 
 * See {@link FalconDriveMotor} for configuration details.
 */
public class Falcon6TurningMotor implements Motor100<Angle100> {
    //TODO Tune ff for amps
    private final double m_gearRatio;
    private static final double kCurrentLimit = 40;

    /**
     * The speed, below which, static friction applies, in motor revolutions per
     * second.
     */
    private static final double staticFrictionSpeedLimitRev_S = 3.5;

    /**
     * Friction feedforward in amps, for when the mechanism is stopped, or nearly
     * so.
     */
    private static final double staticFrictionFFAmps = 0.375;

    /**
     * Friction feedforward in amps, for when the mechanism is moving.
     */
    private static final double dynamicFrictionFFAmps = 0.27;

    /**
     * Velocity feedforward in amps
     */
    private static final double velocityFFAmps_Rev = 0.11;

    /**
     * Placeholder for accel feedforward.
     */
    private static final double accelFFAmps2_Rad = 0;

    /**
     * Proportional feedback coefficient for the controller. The error is measured
     * in sensor units (ticks per 100ms), and the full scale output is 1023.
     */
    //TODO Fix PID
    private static final double outboardP = 0.001;

    private final Telemetry t = Telemetry.get();
    private final TalonFX m_motor;
    private final String m_name;

    /** Current velocity, updated in periodic(). */
    private double m_velocityRev_S;
    /** Current output, updated in periodic() */
    private double m_output;
    /** Current motor error, updated in periodic() */
    private double m_error;

    public Falcon6TurningMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double kGearRatio) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();

        m_motor = new TalonFX(canId);

        // m_motor.configFactoryDefault();
        var talonFXConfigurator = m_motor.getConfigurator();

        TalonFXConfiguration conf = new TalonFXConfiguration();
        talonFXConfigurator.apply(conf);

        // m_motor.setNeutralMode(NeutralMode.Brake);

        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        m_gearRatio = kGearRatio;
        // the serve module steering gear is inverted
        if (motorPhase == MotorPhase.FORWARD) {
            motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            // m_motor.setInverted(InvertType.None);
        } else {
            motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
            // m_motor.setInverted(InvertType.InvertMotorOutput);
        }

        talonFXConfigurator.apply(motorConfigs);

        var currentConfigs = new CurrentLimitsConfigs();
        // i think maybe we don't care about this?
        // currentConfigs.StatorCurrentLimit = kCurrentLimit;
        // currentConfigs.StatorCurrentLimitEnable = true;
        // we're just trying to avoid draining the battery too much.
        currentConfigs.SupplyCurrentLimit = kCurrentLimit;
        currentConfigs.SupplyCurrentLimitEnable = true;
        talonFXConfigurator.apply(currentConfigs);

        // // configure current limits
        // m_motor.configStatorCurrentLimit(
        // new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));
        // m_motor.configSupplyCurrentLimit(
        // new SupplyCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));

        // use integrated sensor for status and PID feedback
        // m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

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
    public void setVelocity(double outputRad_S, double accelRad_S_S) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;
        // double motorRev_100ms = motorRev_S / 10;
        // double motorTick_100ms = motorRev_100ms * ticksPerRevolution;

        double currentMotorRev_S = m_velocityRev_S;
        double frictionFF = frictionFF(currentMotorRev_S, motorRev_S);
        double velocityFF = velocityFF(motorRev_S);
        double accelFF = accelFF(accelRad_S_S);
        double kFF = frictionFF + velocityFF + accelFF;

        VelocityTorqueCurrentFOC v = new VelocityTorqueCurrentFOC(motorRev_S);
        v.FeedForward = kFF;
        v.Acceleration = motorRev_S2;
        m_motor.setControl(v);

        // m_motor.set(ControlMode.Velocity, motorTick_100ms,
        // DemandType.ArbitraryFeedForward, kFF);
        t.log(Level.DEBUG, m_name, "motor input (RPS)", motorRev_S);
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

    @Override
    public void periodic() {
        // m_rawVelocity = m_motor.getSelectedSensorVelocity();
        m_velocityRev_S = m_motor.getVelocity().getValueAsDouble();

        // m_output = m_motor.getMotorOutputPercent();
        m_output = m_motor.getDutyCycle().getValueAsDouble();
        m_error = m_motor.getClosedLoopError().getValueAsDouble();
        t.log(Level.DEBUG, m_name, "velocity (rev_s)", m_velocityRev_S);
        t.log(Level.DEBUG, m_name, "output [-1,1]", m_output);
        t.log(Level.DEBUG, m_name, "error (rev_s)", getErrorRev_S());
    }

    //////////////////////////////////////////////////////////////////

    /** Velocity feedforward in amps */
    private static double velocityFF(double desiredMotorRev_S) {
        return velocityFFAmps_Rev * desiredMotorRev_S;
    }

    /** Frictional feedforward in amps */
    private static double frictionFF(double currentMotorRev_S, double desiredMotorRev_S) {
        double direction = Math.signum(desiredMotorRev_S);
        if (currentMotorRev_S < staticFrictionSpeedLimitRev_S) {
            return staticFrictionFFAmps * direction;
        }
        return dynamicFrictionFFAmps * direction;
    }

    /**
     * Acceleration feedforward in amps
     */
    private static double accelFF(double accelRad_S_S) {
        return accelFFAmps2_Rad * accelRad_S_S;
    }

    private double getErrorRev_S() {
        double errorTick_100ms = m_error;
        double errorRev_100ms = errorTick_100ms;
        return errorRev_100ms * 10;
    }
}
