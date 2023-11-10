package org.team100.distance_sensor;

import org.team100.distance_sensor.subsystems.distance_sensor.NTDistanceSensor;

public class RobotContainer {
    public RobotContainer() {
        new NTDistanceSensor("foo");
    }
}
