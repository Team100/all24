package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.units.Measure100;

public class MockPositionMotor100<T extends Measure100> implements PositionMotor100<T>, GenericTorqueModel {
    public double position = 0;

    @Override
    public void stop() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setPosition(double position, double torque) {
        this.position = position;
    }

    @Override
    public String getGlassName() {
        return "MockPositionMotor100";
    }
}
