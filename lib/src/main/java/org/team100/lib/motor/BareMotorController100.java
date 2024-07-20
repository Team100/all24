package org.team100.lib.motor;

import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BareMotorController100 implements BareMotor {
    /**
     * Very much not calibrated.
     * Say 600 rad/s max so 0.0016?
     */
    private static final double velocityFFDutyCycle_Rad_S = 0.0016;
    private final SupplierLogger m_logger;
    private final MotorController m_motor;

    public BareMotorController100(
            SupplierLogger parent,
            MotorController motorController) {
        m_motor = motorController;
        m_motor.setInverted(true);
        m_logger = parent.child(this);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_logger.logDouble(Level.TRACE, "duty cycle", () -> output);
    }

    /**
     * Velocity kV only, ignores accel and torque.
     */
    @Override
    public void setVelocity(double motorRad_S, double accelRad_S2, double torqueNm) {
        double motorDutyCycle = motorRad_S * velocityFFDutyCycle_Rad_S;
        m_motor.set(motorDutyCycle);
        m_logger.logDouble(Level.TRACE, "duty cycle", () -> motorDutyCycle);
    }

    /** MotorControllers do not support positional control. */
    @Override
    public void setPosition(double position, double velocity, double torque) {
        throw new UnsupportedOperationException();
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void close() {
        // m_motor.close();
    }

    /** MotorControllers do not support velocity measurement. */
    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void periodic() {
        m_logger.logDouble(Level.TRACE, "duty cycle reported", m_motor::get);
    }
}
