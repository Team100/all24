package org.team100.lib.motor.turning;

import org.team100.lib.telemetry.Telemetry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

public class NeoTurningMotor implements TurningMotor {
    public static class Config {
        public int kCurrentLimit = 40;
    }

    private final Config m_config = new Config();
    private final Telemetry t = Telemetry.get();

    private SparkMaxPIDController m_pidController;
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

        t.log(m_name + "/Device ID", m_motor.getDeviceId());
    }

    @Override
    public double get() {
        return m_motor.get();
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
        t.log(m_name + "/Output", output);
    }

    public void setPIDVelocity(double output, double Accel) {
        double motorGearing = 1;
        // I believe there are not any robotics that actually use a neo turning motor I
        // was changing this class to be more ready in case we do
        final ControlType controlType = CANSparkMax.ControlType.kVelocity;
        m_pidController.setReference(motorGearing * output, controlType);
    }

    // THIS DOES NOT ACTUALLY SET PID This is just here for the other turning motors
    // TODO fix this
    public void setPIDPosition(double output) {
        this.set(output);
    }
}
