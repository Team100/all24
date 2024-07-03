package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.TorqueModel;
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

public abstract class CANSparkMotor<T extends Measure100>
        implements DutyCycleMotor100, VelocityMotor100<T>, PositionMotor100<T>, TorqueModel {
    protected final Telemetry.Logger t;
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
        t = Telemetry.get().logger(m_name);
        m_ff = ff;
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, motorPhase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        Rev100.pidConfig(m_pidController, pid);
        t.log(Level.TRACE, "Device ID", m_motor.getDeviceId());
        t.register(Level.TRACE, "P", pid.getP(), this::setP);
        t.register(Level.TRACE, "I", pid.getI(), this::setI);
        t.register(Level.TRACE, "D", pid.getD(), this::setD);
        t.register(Level.TRACE, "IZone", pid.getIZone(), this::setIZone);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.logDouble(Level.TRACE, "Output", output);
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
     * Set motor output using motor quantities (rev/s, rev/s^2, Nm).
     */
    protected void setMotorVelocity(
            double motorRev_S,
            double motorRev_S2,
            double torqueNm) {
        double currentMotorRev_S = m_encoder.getVelocity() / 60;

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);
        double torqueFFVolts = getTorqueFFVolts(torqueNm);

        double kFF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        double motorRev_M = motorRev_S * 60;
        Rev100.warn(() -> m_pidController.setReference(
                motorRev_M, ControlType.kVelocity, 0, kFF, ArbFFUnits.kVoltage));

        t.logDouble(Level.TRACE, "desired speed (rev_s)",()-> motorRev_S);
        t.logDouble(Level.TRACE, "desired accel (rev_s2)",()-> motorRev_S2);
        t.logDouble(Level.TRACE, "friction feedforward (v)", ()->frictionFFVolts);
        t.logDouble(Level.TRACE, "velocity feedforward (v)", ()->velocityFFVolts);
        t.logDouble(Level.TRACE, "accel feedforward (v)", ()->accelFFVolts);
        t.logDouble(Level.TRACE, "torque feedforward (v)", ()->torqueFFVolts);
        log();
    }

    /**
     * Set motor output using motor quantities (rev, Nm).
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    protected void setMotorPosition(
            double motorRev,
            double motorRev_S,
            double torqueNm) {
        double currentMotorRev_S = m_encoder.getVelocity() / 60;

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double torqueFFVolts = getTorqueFFVolts(torqueNm);

        double kFF = frictionFFVolts + velocityFFVolts + torqueFFVolts;

        Rev100.warn(() -> m_pidController.setReference(
                motorRev, ControlType.kPosition, 0, kFF, ArbFFUnits.kVoltage));

        t.logDouble(Level.TRACE, "desired position (rev)", ()->motorRev);
        t.logDouble(Level.TRACE, "desired speed (rev_s)", ()->motorRev_S);
        t.logDouble(Level.TRACE, "friction feedforward (v)", ()->frictionFFVolts);
        t.logDouble(Level.TRACE, "velocity feedforward (v)", ()->velocityFFVolts);
        t.logDouble(Level.TRACE, "torque feedforward (v)",()-> torqueFFVolts);
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
    public void resetEncoderPosition() {
        Rev100.warn(() -> m_encoder.setPosition(0));
    }

    /**
     * Set integrated sensor position in rotations.
     */
    public void setEncoderPosition(double motorPositionRev) {
        Rev100.warn(() -> m_encoder.setPosition(motorPositionRev));
    }

    protected void log() {
        t.logDouble(Level.TRACE, "position (rev)",()-> m_encoder.getPosition());
        t.logDouble(Level.TRACE, "velocity (rev_s)",()-> m_encoder.getVelocity() / 60);
        t.logDouble(Level.TRACE, "velocity (RPM)",()-> m_encoder.getVelocity());
        t.logDouble(Level.TRACE, "current (A)", ()->m_motor.getOutputCurrent());
        t.logDouble(Level.TRACE, "duty cycle", ()->m_motor.getAppliedOutput());
        t.logDouble(Level.TRACE, "torque (Nm)", ()->getMotorTorque());
        t.logDouble(Level.TRACE, "temperature (C)", ()->m_motor.getMotorTemperature());
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
