package frc.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Mirrors the input. */
public class ExampleSubsystem extends Subsystem {
    private double state;

    public ExampleSubsystem() {
        state = 0;
    }

    public void setState(double v) {
        state = v;
    }

    public double getState() {
        return state;
    }
}
