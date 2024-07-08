package org.team100.lib.motor.turning;

import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Calibrated angle motor wrapping any MotorController.
 */
public class TurningMotorController100 implements BareMotor, GenericTorqueModel {
    /**
     * Very much not calibrated.
     * Say 600 rad/s max so 0.0016?
     */
    private static final double velocityFFDutyCycle_Rad_S = 0.0016;
    private final Logger m_logger;
    private final MotorController m_motor;

    public TurningMotorController100(
            Logger parent,
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

    @Override
    public void stop() {
        m_motor.stopMotor();
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

    @Override
    public void close() {
        // m_motor.close();
    }

    /** MotorControllers do not support positional control. */
    @Override
    public void setPosition(double position, double velocity, double torque) {
        throw new UnsupportedOperationException();
    }

    /** MotorControllers do not support velocity measurement. */
    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException();
    }
}
