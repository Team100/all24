package org.team100.lib.motor;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motor.model.TorqueModel;
import org.team100.lib.units.Measure100;

/**
 * The Motor100 class is for calibrated motor commands.
 */
public interface Motor100<T extends Measure100> extends Glassy, TorqueModel {

    /**
     * Open-loop duty cycle control.
     * 
     * @param output in range [-1, 1]
     */
    void setDutyCycle(double output);

    /**
     * Closed-loop velocity control.
     * 
     * @param velocity setpoint, T/s,
     * @param accel    used for feedforward, T/s/s
     */
    void setVelocity(double velocity, double accel);

    /**
     * Includes feedforward in Nm.
     * TODO: make feedforward use the Measure units (Nm or N).
     * 
     * @param velocity
     * @param accel
     * @param torque   feedforward torque in Nm.
     */
    void setVelocity(double velocity, double accel, double torque);

    /**
     * @return Current applied torque. Used for drive/steer decoupling.
     */
    double getTorque();

    void stop();

    /**
     * For test cleanup.
     */
    void close();

    @Override
    default String getGlassName() {
        return "Motor100";
    }
}