package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class AngularNeoProxy implements BareMotor, NeoTorqueModel {
    private final Logger m_logger;

    private final CANSparkMax m_motor;

    public AngularNeoProxy(
            Logger parent,
            int canId,
            IdleMode idleMode,
            int currentLimit) {
        m_logger = parent.child(this);
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, idleMode, MotorPhase.FORWARD, 20);
        Rev100.currentConfig(m_motor, currentLimit);
    }

    private void set(double speed) {
        m_motor.set(speed);
        m_logger.logDouble(Level.TRACE, "DUTY", m_motor::getAppliedOutput);
        m_logger.logDouble(Level.TRACE, "AMPS", m_motor::getOutputCurrent);
    }

    @Override
    public void setDutyCycle(double output) {
        set(output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        //
    }

    @Override
    public void setPosition(double position, double velocity, double torque) {
        //
    }

    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException();
    }
}
