package org.team100.lib.motor.turning;

public class MockTurningMotor implements TurningMotor {
    public double turningOutput = 0;
    public double turningVelocity = 0;

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setDutyCycle(double output) {
        turningOutput = output;
    }

    @Override
    public void setVelocity(double output, double outputAccel) {
        turningVelocity = output;
    }

}
