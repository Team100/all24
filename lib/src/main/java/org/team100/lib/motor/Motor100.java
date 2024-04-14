package org.team100.lib.motor;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.units.Measure100;

/**
 * The Motor100 class is for calibrated motor commands.
 */
public interface Motor100<T extends Measure100> extends Glassy {

    /**
     * Open-loop duty cycle control.
     * 
     * @deprecated we shouldn't use duty cycle anywhere.
     * 
     * @param output in range [-1, 1]
     */
    @Deprecated
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
     * @param velocity
     * @param accel
     * @param torque feedforward torque in Nm.
     */
    void setVelocity(double velocity, double accel, double torque);

    /**
     * @return Current applied torque.  Used for drive/steer decoupling.
     */
    double getTorque();

    void stop();

    /**
     * For test cleanup.
     */
    void close();

    /**
     * Used to collect measurements once per cycle, to save time and improve
     * consistency.
     */
    default void periodic() {}
    @Override
    default String getGlassName() {
        return "Motor100";
    }
}