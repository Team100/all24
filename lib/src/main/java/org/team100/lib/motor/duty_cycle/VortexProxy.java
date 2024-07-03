package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.motor.model.NeoVortexTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

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
    private final Telemetry.Logger t;
    private final String m_name;
    private final CANSparkFlex m_motor;
    private final RelativeEncoder m_encoder;

    public VortexProxy(
            String name,
            int canId,
            MotorPhase phase,
            int currentLimit) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = new CANSparkFlex(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, phase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
    }

    private void set(double speed) {
        m_motor.set(speed);
        t.logDouble(Level.DEBUG, "current (A)", () -> m_motor.getOutputCurrent());
        t.logDouble(Level.DEBUG, "duty cycle", () -> m_motor.getAppliedOutput());
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
