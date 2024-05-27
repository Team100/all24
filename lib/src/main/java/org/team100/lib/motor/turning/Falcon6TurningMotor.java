package org.team100.lib.motor.turning;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    private static final double kCurrentLimit = 10;

    private final Feedforward100 m_ff;

    private final Telemetry t = Telemetry.get();
    private final TalonFX m_motor;
    private final double m_gearRatio;
    private final String m_name;

    public Falcon6TurningMotor(
            String name,
            int canId,
            MotorPhase motorPhase,
            double kGearRatio,
            PIDConstants lowLevelVelocityConstants,
            Feedforward100 ff) {
        m_ff = ff;

        if (name.startsWith("/"))
            throw new IllegalArgumentException();

        m_motor = new TalonFX(canId);

        TalonFXConfigurator talonFXConfigurator = m_motor.getConfigurator();

        TalonFXConfiguration conf = new TalonFXConfiguration();
        talonFXConfigurator.apply(conf);

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        m_gearRatio = kGearRatio;
        // the serve module steering gear is inverted
        if (motorPhase == MotorPhase.FORWARD) {
            motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        }

        talonFXConfigurator.apply(motorConfigs);

        // Avoid draining the battery too much.
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.SupplyCurrentLimit = kCurrentLimit;
        currentConfigs.SupplyCurrentLimitEnable = true;
        talonFXConfigurator.apply(currentConfigs);

        m_motor.getVelocity().setUpdateFrequency(50);

        // set slot 0 gains
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0;
        slot0Configs.kP = lowLevelVelocityConstants.getP();
        slot0Configs.kI = lowLevelVelocityConstants.getI();
        slot0Configs.kD = lowLevelVelocityConstants.getD();
        // apply gains, 50 ms total timeout
        m_motor.getConfigurator().apply(slot0Configs, 0.050);

        m_name = Names.append(name, this);
        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceID());
    }

    @Override
    public void setDutyCycle(double output) {
        DutyCycleOut d = new DutyCycleOut(output);
        m_motor.setControl(d);
        t.log(Level.TRACE, m_name, "desired duty cycle [-1,1]", output);
        log();
    }

    /**
     * Supports accel feedforward.
     */
    public void setVelocity(double outputRad_S, double accelRad_S_S) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double currentMotorRev_S = m_motor.getVelocity().getValueAsDouble();
        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts;

        VelocityVoltage v = new VelocityVoltage(motorRev_S);
        v.FeedForward = kFFVolts;
        v.Acceleration = motorRev_S2;
        m_motor.setControl(v);

        t.log(Level.TRACE, m_name, "motor input (RPS)", motorRev_S);
        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        log();

    }

    /**
     * Supports accel feedforward.
     */
    public void setVelocity(double outputRad_S, double accelRad_S_S, double torqueNm) {
        double outputRev_S = outputRad_S / (2 * Math.PI);
        double wheelRev_S2 = accelRad_S_S / (2 * Math.PI);
        double motorRev_S = outputRev_S * m_gearRatio;
        double motorRev_S2 = wheelRev_S2 * m_gearRatio;

        double currentMotorRev_S = m_motor.getVelocity().getValueAsDouble();
        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double torqueFFAmps = torqueNm / kTNm_amp;
        double torqueFFVolts = torqueFFAmps * kROhms;

        double kFFVolts = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        VelocityVoltage v = new VelocityVoltage(motorRev_S);
        v.FeedForward = kFFVolts;
        v.Acceleration = motorRev_S2;
        m_motor.setControl(v);

        t.log(Level.TRACE, m_name, "motor input (RPS)", motorRev_S);
        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        log();
    }

    @Override
    public double getTorque() {
        StatusSignal<Double> statorCurrentAmpsStatus = m_motor.getTorqueCurrent();
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

    public void log() {
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_motor.getVelocity().getValueAsDouble());
        t.log(Level.TRACE, m_name, "output [-1,1]", m_motor.getDutyCycle().getValueAsDouble());
        t.log(Level.TRACE, m_name, "error (rev_s)", m_motor.getClosedLoopError().getValueAsDouble());
        t.log(Level.TRACE, m_name, "current (A)", m_motor.getSupplyCurrent().getValueAsDouble());
    }

    //////////////////////////////////////////////////////////////////

}
