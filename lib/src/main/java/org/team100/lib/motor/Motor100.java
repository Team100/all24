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
     * Closed-loop velocity control with acceleration and holding torque.
     * 
     * @param velocity desired velocity in m/s or rad/s.
     * @param accel    desired accel in m/s^2 or rad/s^2
     * @param torque   desired torque in N or Nm, for gravity compensation or
     *                 holding.
     */
    void setVelocity(double velocity, double accel, double torque);

    /**
     * @return Current applied torque in N or Nm, for drive/steer
     *         decoupling.
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