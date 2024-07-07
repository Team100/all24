package org.team100.lib.motor.drive;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Calibrated distance motor using any MotorController.
 */
public class DriveMotorController100 implements Motor100<Distance100>, GenericTorqueModel {
    /**
     * Very much not calibrated. Say 100 rev/s max so 0.01?
     */
    private static final double velocityFFDutyCycle_Rev_S = 0.01;
    private final Logger m_logger;
    private final MotorController m_motor;
    private final double m_gearRatio;
    private final double m_wheelDiameter;

    public DriveMotorController100(
            Logger parent,
            MotorController motorController,
            double kDriveReduction,
            double wheelDiameter) {
        m_motor = motorController;
        m_motor.setInverted(true);
        m_logger = parent.child(this);
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;
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
     * Velocity kV only.
     */
    @Override
    public void setVelocity(double outputM_S, double accelM_S2, double torqueN) {
        double wheelRev_S = outputM_S / (m_wheelDiameter * Math.PI);
        double motorRev_S = wheelRev_S * m_gearRatio;
        double motorDutyCycle = motorRev_S * velocityFFDutyCycle_Rev_S;
        m_motor.set(motorDutyCycle);
        m_logger.logDouble(Level.TRACE, "duty cycle", () -> motorDutyCycle);
    }

    @Override
    public void close() {
        // m_motor.close();
    }

    @Override
    public void setPosition(double position, double velocity, double torque) {
        //
    }
}
