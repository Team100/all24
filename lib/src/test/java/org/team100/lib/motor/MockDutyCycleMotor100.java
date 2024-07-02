package org.team100.lib.motor;

import org.team100.lib.motor.model.GenericTorqueModel;
import org.team100.lib.units.Measure100;

public class MockDutyCycleMotor100<T extends Measure100> implements DutyCycleMotor100, GenericTorqueModel {
    public double output = 0;

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void stop() {
        this.output = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public String getGlassName() {
        return "MockDutyCycleMotor100";
    }
}
