package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public abstract class CANSparkMotor implements BareMotor {
    protected final SupplierLogger m_logger;
    protected final Feedforward100 m_ff;
    protected final CANSparkBase m_motor;
    protected final RelativeEncoder m_encoder;
    protected final SparkPIDController m_pidController;

    protected CANSparkMotor(
            SupplierLogger parent,
            CANSparkBase motor,
            MotorPhase motorPhase,
            int currentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        m_motor = motor;
        m_logger = parent.child(this);
        m_ff = ff;
        // make config synchronous so we can see the errors
        Rev100.crash(() -> m_motor.setCANTimeout(500));
        Rev100.baseConfig(m_motor);
        Rev100.motorConfig(m_motor, IdleMode.kBrake, motorPhase, 20);
        Rev100.currentConfig(m_motor, currentLimit);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        Rev100.pidConfig(m_pidController, pid);
        // make everything after this asynchronous.
        // NOTE: this makes error-checking not work at all.
        Rev100.crash(() -> m_motor.setCANTimeout(0));
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_logger.logDouble(Level.TRACE, "Duty Cycle", () -> output);
        log();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        Rev100.currentConfig(m_motor, currentA);
    }

    /**
     * Use outboard PID control to hold the given velocty, with acceleration and
     * torque feedforwards.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorAccelRad_S2, double motorTorqueNm) {
        double motorRev_S = motorRad_S / (2 * Math.PI);
        double motorRev_S2 = motorAccelRad_S2 / (2 * Math.PI);
        double currentMotorRev_S = m_encoder.getVelocity() / 60;

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double accelFFVolts = m_ff.accelFFVolts(motorRev_S2);
        double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

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
     * Use outboard PID control to hold the given position, with velocity and torque
     * feedforwards.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setPosition(double motorPositionRad, double motorVelocityRad_S, double motorTorqueNm) {
        double motorRev = motorPositionRad / (2 * Math.PI);
        double motorRev_S = motorVelocityRad_S / (2 * Math.PI);
        double currentMotorRev_S = m_encoder.getVelocity() / 60;

        double frictionFFVolts = m_ff.frictionFFVolts(currentMotorRev_S, motorRev_S);
        double velocityFFVolts = m_ff.velocityFFVolts(motorRev_S);
        double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

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

    @Override
    public double getVelocityRad_S() {
        return getRateRPM() * 2 * Math.PI / 60;
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        setEncoderPosition(positionRad / (2 * Math.PI));
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    public double getMotorTorque() {
        return m_motor.getOutputCurrent() * kTNm_amp();
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
        m_logger.logInt(Level.TRACE, "Device ID", m_motor::getDeviceId);
        m_logger.logDouble(Level.TRACE, "position (rev)", m_encoder::getPosition);
        m_logger.logDouble(Level.TRACE, "velocity (rev_s)", () -> m_encoder.getVelocity() / 60);
        m_logger.logDouble(Level.TRACE, "velocity (RPM)", m_encoder::getVelocity);
        m_logger.logDouble(Level.TRACE, "current (A)", m_motor::getOutputCurrent);
        m_logger.logDouble(Level.TRACE, "duty cycle", m_motor::getAppliedOutput);
        m_logger.logDouble(Level.TRACE, "torque (Nm)", this::getMotorTorque);
        m_logger.logDouble(Level.TRACE, "temperature (C)", m_motor::getMotorTemperature);
    }
}
