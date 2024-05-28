package org.team100.lib.motor.turning;

import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Calibrated angle motor wrapping any MotorController.
 */
public class TurningMotorController100 implements Motor100<Angle100>, GenericTorqueModel {
    /**
     * Very much not calibrated.
     * Say 600 rad/s max so 0.0016?
     */
    private static final double velocityFFDutyCycle_Rad_S = 0.0016;
    private final Telemetry t = Telemetry.get();
    private final MotorController m_motor;
    private final String m_name;
    private final double m_gearRatio;

    public TurningMotorController100(
            String name,
            MotorController motorController,
            double kDriveReduction) {
        if (name.startsWith("/"))
            throw new IllegalArgumentException();
        m_motor = motorController;
        m_motor.setInverted(true);
        m_name = Names.append(name, this);
        m_gearRatio = kDriveReduction;
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        t.log(Level.TRACE, m_name, "duty cycle", output);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Velocity is just kV feedforward and that's all.
     * 
     * @param outputRad_S times kV
     * @param accelRad_S2 ignored
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2) {
        double motorRad_S = outputRad_S * m_gearRatio;
        double motorDutyCycle = motorRad_S * velocityFFDutyCycle_Rad_S;
        m_motor.set(motorDutyCycle);
        t.log(Level.TRACE, m_name, "duty cycle", motorDutyCycle);
    }

    /**
     * @param outputRad_S times kv
     * @param accelRad_S2 ignored
     * @param torque      ignored
     */
    @Override
    public void setVelocity(double outputRad_S, double accelRad_S2, double torque) {
        setVelocity(outputRad_S, accelRad_S2);
    }

    @Override
    public double getTorque() {
        throw new UnsupportedOperationException("Unimplemented method 'getTorque'");
    }

    @Override
    public void close() {
        // m_motor.close();
    }
}
