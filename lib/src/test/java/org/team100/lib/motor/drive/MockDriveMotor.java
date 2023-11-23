package org.team100.lib.motor.drive;

public class MockDriveMotor implements DriveMotor {
    public double output = 0;
    public double velocity = 0;

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

}
