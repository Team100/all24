package org.team100.lib.motor.drive;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Calibrated distance motor using any MotorController.
 */
public class DriveMotorController100 implements Motor100<Distance100>, GenericTorqueModel {
    /**
     * Very much not calibrated. Say 100 rev/s max so 0.01?
     */
    private static final double velocityFFDutyCycle_Rev_S = 0.01;
    private final Telemetry.Logger t;
    private final MotorController m_motor;
    private final String m_name;
    private final double m_gearRatio;
    private final double m_wheelDiameter;

    public DriveMotorController100(
            String name,
            Logger parent,
            MotorController motorController,
            double kDriveReduction,
            double wheelDiameter) {
        m_motor = motorController;
        m_motor.setInverted(true);
        m_name = Names.append(name, this);
        t = Telemetry.get().logger(m_name, parent);
        m_wheelDiameter = wheelDiameter;
        m_gearRatio = kDriveReduction;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.logDouble(Level.TRACE, "duty cycle", ()->output);
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
        t.logDouble(Level.TRACE, "duty cycle",()-> motorDutyCycle);
    }

    @Override
    public void close() {
        // m_motor.close();
    }
}
