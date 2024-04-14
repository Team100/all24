package org.team100.lib.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MockMotorController implements MotorController {
    public double speed;
    public boolean inverted;

    @Override
    public void set(double speed) {
        this.speed = speed;
    }

    @Override
    public double get() {
        return speed;
    }

    @Override
    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void disable() {
        //
    }

    @Override
    public void stopMotor() {
        speed = 0;
    }
}
