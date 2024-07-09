package org.team100.lib.motor;

public class MockBareMotor implements BareMotor {
    public double output = 0;
    public double velocity = 0;
    public double position = 0;

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVelocity(double velocityRad_S, double accel, double torque) {
        this.velocity = velocityRad_S;
    }

    @Override
    public void setPosition(double position, double velocity, double torque) {
        this.position = position;
    }

    @Override
    public void stop() {
        this.output = 0;
        this.velocity = 0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return this.velocity;
    }

    @Override
    public void setEncoderPositionRad(double positionRad) {
        this.position = positionRad;
    }

}
