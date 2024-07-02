package org.team100.lib.motor.duty_cycle;

import org.team100.lib.motor.DutyCycleMotor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * A very simple wrapper around a CANSparkMax that only supports duty-cycle
 * output.
 * 
 * This makes the code that uses it easier to test.
 */
public class NeoProxy implements DutyCycleMotor100, NeoTorqueModel {
    private final Telemetry.Logger t;
    private final String m_name;

    private final CANSparkMax m_motor;

    public NeoProxy(
            String name,
            int canId,
            IdleMode idleMode,
            int currentLimit) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, idleMode, MotorPhase.FORWARD, 20);
        Rev100.currentConfig(m_motor, currentLimit);
    }

    private void set(double speed) {
        m_motor.set(speed);
        t.log(Level.TRACE, "DUTY", m_motor.getAppliedOutput());
        t.log(Level.TRACE, "AMPS", m_motor.getOutputCurrent());
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
}
