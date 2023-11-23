package org.team100.lib.motor.turning;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

public class NeoTurningMotor implements TurningMotor {
    public static class Config {
        public int kCurrentLimit = 40;
        public double kMotorGearing = 1;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();

    private final SparkMaxPIDController m_pidController;
    private final CANSparkMax m_motor;
    private final String m_name;

    public NeoTurningMotor(String name, int canId) {
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(m_config.kCurrentLimit);
        m_pidController = m_motor.getPIDController();
        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setP(.1);
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
        return m_motor.get();
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.DEBUG, m_name + "/Output", output);
    }

    public void setVelocity(double outputRadiansPerSec, double Accel) {
        t.log(Level.DEBUG, m_name + "/Output", outputRadiansPerSec);
        m_pidController.setReference(m_config.kMotorGearing * outputRadiansPerSec, CANSparkMax.ControlType.kVelocity);
        throw new UnsupportedOperationException("NEO closed loop velocity control is uncalibrated.");
    }
}
