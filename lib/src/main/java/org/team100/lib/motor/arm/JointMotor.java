package org.team100.lib.motor.arm;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Rev100;
import org.team100.lib.motor.model.NeoTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * Arm motor from 2023.
 * 
 * Velocity is feedforward only.
 * 
 * Note the absence of gear ratio; this should be added before using this class.
 */
public class JointMotor implements Motor100<Angle100>, NeoTorqueModel {
    /** Very much not calibrated. */
    private static final double kV = 0.1;
    private final Telemetry.Logger t;
    private final CANSparkMax m_motor;
    private final String m_name;

    /**
     * @param name         may not start with a slash.
     * @param canId
     * @param currentLimit
     */
    public JointMotor(String name, int canId, int currentLimit) {
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name);
        m_motor = new CANSparkMax(canId, MotorType.kBrushless);
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, MotorPhase.FORWARD, 10);
        Rev100.currentConfig(m_motor, currentLimit);

        t.log(Level.TRACE, "Device ID", m_motor.getDeviceId());
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, "Duty Cycle", output);
    }

    /**
     * Velocity kV only.
     */
    @Override
    public void setVelocity(double velocity, double accel, double torqueNm) {
        m_motor.set(kV * velocity);
        t.log(Level.TRACE, "Velocity", velocity);
        t.log(Level.TRACE, "Accel", accel);
        t.log(Level.TRACE, "Desired torque Nm", torqueNm);
        t.log(Level.TRACE, "Actual torque Nm", getTorque());
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    private double getTorque() {
        return kTNm_amp() * m_motor.getOutputCurrent();
    }

}
