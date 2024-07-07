package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.units.Distance100;

public class MockLinearVelocityMotor100 implements VelocityMotor100<Distance100>, GenericTorqueModel {
    public double output = 0;
    public double velocity = 0;

    @Override
    public void stop() {
        this.output = 0;
    }

    /**
     * Velocity only.
     */
    @Override
    public void setVelocity(double velocity, double accel, double torque) {
        this.velocity = velocity;
    }

    @Override
    public void close() {
        //
    }
    
}
