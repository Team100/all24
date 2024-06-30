package org.team100.lib.motion.components;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.units.Measure100;

public interface PositionServo<T extends Measure100> extends Glassy {

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * @param goal For distance, use meters, For angle, use radians.
     * @param feedForwardTorque used for gravity compensation
     */
    void setPosition(double goal, double feedForwardTorqueNm);

    /**
     * @return Current position measurement. For distance this is meters, for angle
     *         this is radians.
     */
    double getPosition();

    double getVelocity();

    boolean atSetpoint();

    boolean atGoal();

    double getGoal();

    void stop();

    void close();

    State100 getSetpoint();

    @Override
    default String getGlassName() {
        return "PositionServo";
    }

}