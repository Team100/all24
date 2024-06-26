package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public abstract class CANSparkMotor<T extends Measure100> implements Motor100<T> {
    protected final Telemetry t = Telemetry.get();
    private final String m_name;
    protected final Feedforward100 m_ff;
    protected final CANSparkBase m_motor;
    protected final RelativeEncoder m_encoder;
    protected final SparkPIDController m_pidController;

    protected CANSparkMotor(
            String name,
            CANSparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        m_motor = motor;
        m_name = Names.append(name, this);
        m_ff = ff;
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, motorPhase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        Rev100.pidConfig(m_pidController, pid);
        t.log(Level.TRACE, m_name, "Device ID", m_motor.getDeviceId());
        t.register(Level.TRACE, m_name, "P", pid.getP(), this::setP);
        t.register(Level.TRACE, m_name, "I", pid.getI(), this::setI);
        t.register(Level.TRACE, m_name, "D", pid.getD(), this::setD);
        t.register(Level.TRACE, m_name, "IZone", pid.getIZone(), this::setIZone);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "Output", output);
        log();
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    public double getMotorTorque() {
        return m_motor.getOutputCurrent() * kTNm_amp();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Bypass the gear ratio, wheel diameter, etc, and set the motor directly.
     */
    public void setMotorVelocity(double motorRev_S, double motorRev_S2, double torqueNm) {
        double motorRev_M = motorRev_S * 60;

        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double frictionFFVolts = m_ff.frictionFFVolts(m_encoder.getVelocity() / 60, motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);

        double torqueFFAmps = torqueNm / kTNm_amp();
        double torqueFFVolts = torqueFFAmps * kROhms();

        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        Rev100.warn(() -> m_pidController.setReference(motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.log(Level.TRACE, m_name, "friction feedforward volts", frictionFFVolts);
        t.log(Level.TRACE, m_name, "velocity feedforward volts", velocityFFVolts);
        t.log(Level.TRACE, m_name, "accel feedforward volts", accelFFVolts);
        t.log(Level.TRACE, m_name, "torque feedforward volts", torqueFFVolts);
        t.log(Level.TRACE, m_name, "desired speed (rev_s)", motorRev_S);
        log();
    }

    /**
     * @return integrated sensor position in rotations.
     */
    public double getPositionRot() {
        // just reads the most-recently received message, so we don't need to cache it
        return m_encoder.getPosition();
    }

    /**
     * @return integrated sensor velocity in RPM
     */
    public double getRateRPM() {
        // just reads the most-recently received message, so we don't need to cache it
        return m_encoder.getVelocity();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetPosition() {
        Rev100.warn(() -> m_encoder.setPosition(0));
    }

    protected void log() {
        t.log(Level.TRACE, m_name, "position (rev)", m_encoder.getPosition());
        t.log(Level.TRACE, m_name, "velocity (rev_s)", m_encoder.getVelocity() / 60);
        t.log(Level.TRACE, m_name, "velocity (RPM)", m_encoder.getVelocity());
        t.log(Level.TRACE, m_name, "current (A)", m_motor.getOutputCurrent());
        t.log(Level.TRACE, m_name, "duty cycle", m_motor.getAppliedOutput());
        t.log(Level.TRACE, m_name, "torque (Nm)", getMotorTorque());
        t.log(Level.TRACE, m_name, "temperature (C)", m_motor.getMotorTemperature());
    }

    private void setP(double p) {
        Rev100.warn(() -> m_pidController.setP(p));
    }

    private void setI(double i) {
        Rev100.warn(() -> m_pidController.setI(i));
    }

    private void setD(double d) {
        Rev100.warn(() -> m_pidController.setD(d));
    }

    private void setIZone(double iz) {
        Rev100.warn(() -> m_pidController.setIZone(iz));
    }

}
