package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.model.TorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Measure100;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public abstract class CANSparkMotor<T extends Measure100>
        implements DutyCycleMotor100, VelocityMotor100<T>, PositionMotor100<T>, TorqueModel {
    protected final Logger m_logger;
    protected final Feedforward100 m_ff;
    protected final CANSparkBase m_motor;
    protected final RelativeEncoder m_encoder;
    protected final SparkPIDController m_pidController;

    protected CANSparkMotor(
            Logger parent,
            CANSparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        m_motor = motor;
        m_logger = parent.child(this);
        m_ff = ff;
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, motorPhase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        Rev100.pidConfig(m_pidController, pid);
        m_logger.log(Level.TRACE, "Device ID", m_motor.getDeviceId());
        m_logger.register(Level.TRACE, "P", pid.getP(), this::setP);
        m_logger.register(Level.TRACE, "I", pid.getI(), this::setI);
        m_logger.register(Level.TRACE, "D", pid.getD(), this::setD);
        m_logger.register(Level.TRACE, "IZone", pid.getIZone(), this::setIZone);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_logger.logDouble(Level.TRACE, "Output", () -> output);
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

        m_logger.logDouble(Level.TRACE, "desired speed (rev_s)", () -> motorRev_S);
        m_logger.logDouble(Level.TRACE, "desired accel (rev_s2)", () -> motorRev_S2);
        m_logger.logDouble(Level.TRACE, "friction feedforward (v)", () -> frictionFFVolts);
        m_logger.logDouble(Level.TRACE, "velocity feedforward (v)", () -> velocityFFVolts);
        m_logger.logDouble(Level.TRACE, "accel feedforward (v)", () -> accelFFVolts);
        m_logger.logDouble(Level.TRACE, "torque feedforward (v)", () -> torqueFFVolts);
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

        m_logger.logDouble(Level.TRACE, "desired position (rev)", () -> motorRev);
        m_logger.logDouble(Level.TRACE, "desired speed (rev_s)", () -> motorRev_S);
        m_logger.logDouble(Level.TRACE, "friction feedforward (v)", () -> frictionFFVolts);
        m_logger.logDouble(Level.TRACE, "velocity feedforward (v)", () -> velocityFFVolts);
        m_logger.logDouble(Level.TRACE, "torque feedforward (v)", () -> torqueFFVolts);
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
        m_logger.logDouble(Level.TRACE, "position (rev)", m_encoder::getPosition);
        m_logger.logDouble(Level.TRACE, "velocity (rev_s)", () -> m_encoder.getVelocity() / 60);
        m_logger.logDouble(Level.TRACE, "velocity (RPM)", m_encoder::getVelocity);
        m_logger.logDouble(Level.TRACE, "current (A)", m_motor::getOutputCurrent);
        m_logger.logDouble(Level.TRACE, "duty cycle",  m_motor::getAppliedOutput);
        m_logger.logDouble(Level.TRACE, "torque (Nm)", this::getMotorTorque);
        m_logger.logDouble(Level.TRACE, "temperature (C)",  m_motor::getMotorTemperature);
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
