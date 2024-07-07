package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.units.Angle100;

public class MockAngularVelocityMotor100 implements VelocityMotor100<Angle100>, GenericTorqueModel {
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
