package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.motor.model.NeoVortexTorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/**
 * Very simple wrapper around SparkFlex for testing.
 * 
 * TODO: remove this class, use NeoVortexDriveEncoder instead.
 */
public class VortexProxy implements DutyCycleMotor100, NeoVortexTorqueModel {
    private final Logger m_logger;
    private final CANSparkFlex m_motor;
    private final RelativeEncoder m_encoder;

    public VortexProxy(
            Logger parent,
            int canId,
            MotorPhase phase,
            int currentLimit) {
        m_logger = parent.child(this);
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, phase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
    }

    private void set(double speed) {
        m_motor.set(speed);
        m_logger.logDouble(Level.DEBUG, "current (A)", m_motor::getOutputCurrent);
        m_logger.logDouble(Level.DEBUG, "duty cycle", m_motor::getAppliedOutput);
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

    double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    double getPositionRot() {
        return m_encoder.getPosition();
    }

    void resetPosition() {
        m_encoder.setPosition(0);
    }
}
