package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconTurningMotor implements Motor100<Angle> {
    private static final double ticksPerRevolution = 2048;
    private static final double gearRatio = 10.29;
    private static final double kCurrentLimit = 40;

    private final Telemetry t = Telemetry.get();
    private final WPI_TalonFX m_motor;
    private final String m_name;

    public FalconTurningMotor(String name, int canId) {
        m_motor = new WPI_TalonFX(canId);
        m_motor.configFactoryDefault();
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.setInverted(InvertType.InvertMotorOutput);
        m_motor.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.config_kF(0, 0);
        m_motor.config_kP(0, 0.1);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
        m_motor.configVoltageCompSaturation(11);
        m_motor.enableVoltageCompensation(true);

        m_name = String.format("/%s/Falcon Turning Motor", name);
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    public void setVelocity(double outputRadiansPerSec, double outputRadiansPerSecPerSec) {
        double motorVelocityRotsPerSec = m_motor.getSelectedSensorVelocity() / (ticksPerRevolution / 10 * gearRatio);

        double revolutionsPerSec = outputRadiansPerSec / (2 * Math.PI);
        // TODO: use accel
        double revolutionsPerSec2 = outputRadiansPerSecPerSec / (2 * Math.PI);
        double revsPer100ms = revolutionsPerSec / 10;
        double ticksPer100ms = revsPer100ms * ticksPerRevolution;
        DemandType type = DemandType.ArbitraryFeedForward;
        double Kn = 0.1136;
        // double Ke = 0.068842;
        double Ks = .027;
        double VSat = 11;
        if (motorVelocityRotsPerSec < .1) {
            Ks = 0.0375;
        }

        double kFF = (Kn * revolutionsPerSec + Ks * Math.signum(revolutionsPerSec)) * gearRatio / VSat;
        m_motor.set(ControlMode.Velocity, ticksPer100ms * gearRatio, type, kFF);
        t.log(Level.DEBUG, m_name + "/Actual Velocity", m_motor.getClosedLoopError() / (ticksPerRevolution / 10));
        t.log(Level.DEBUG, m_name + "/Desired Velocity", m_motor.get());

        log();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        log();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    private void log() {
        t.log(Level.DEBUG, m_name + "/Output", get());
        t.log(Level.DEBUG, m_name + "/Error", m_motor.getClosedLoopError() / (ticksPerRevolution / 10));
    }

}
