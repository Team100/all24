package org.team100.frc2023.subsystems.game_piece_detection.distance_sensors;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTDistanceSensor extends DistanceSensor {
    private final DoubleSubscriber ntVar;

    private double distanceCentimeters;

    public NTDistanceSensor(String networkTableVariable) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable vision_table = inst.getTable("Distances");
        ntVar = vision_table.getDoubleTopic(networkTableVariable).subscribe(-2.0);
    }

    @Override
    public void periodic() {
        double rawValue = ntVar.get();
        if (rawValue != distanceCentimeters) {
            distanceCentimeters = rawValue;
        }
    }

    @Override
    public double getCentimeters() {
        return distanceCentimeters;
    }
}