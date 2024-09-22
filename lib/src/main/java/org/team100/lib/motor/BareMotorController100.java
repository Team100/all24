package org.team100.lib.motor;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BareMotorController100 implements BareMotor {
    /**
     * Very much not calibrated.
     * Say 600 rad/s max so 0.0016?
     */
    private static final double velocityFFDutyCycle_Rad_S = 0.0016;
    private final MotorController m_motor;
    private final DoubleSupplierLogger2 m_log_duty;
    private final DoubleSupplierLogger2 m_log_reported;

    public BareMotorController100(
            SupplierLogger2 parent,
            MotorController motorController) {
        m_motor = motorController;
        m_motor.setInverted(true);
        SupplierLogger2 child = parent.child(this);
        m_log_duty = child.doubleLogger(Level.TRACE, "duty cycle");
        m_log_reported = child.doubleLogger(Level.TRACE, "duty cycle reported");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_log_duty.log(() -> output);
    }

    /**
     * Velocity kV only, ignores accel and torque.
     */
    @Override
    public void setVelocity(double motorRad_S, double accelRad_S2, double torqueNm) {
        double motorDutyCycle = motorRad_S * velocityFFDutyCycle_Rad_S;
        m_motor.set(motorDutyCycle);
        m_log_duty.log(() -> motorDutyCycle);
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
        m_log_reported.log(m_motor::get);
    }
}
