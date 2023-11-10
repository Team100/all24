package org.team100.distance_sensor.subsystems.distance_sensor;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class DistanceSensor extends Subsystem {
    public abstract double getCentimeters();
}
