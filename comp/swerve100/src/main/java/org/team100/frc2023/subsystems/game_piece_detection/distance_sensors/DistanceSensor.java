package org.team100.frc2023.subsystems.game_piece_detection.distance_sensors;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class DistanceSensor extends Subsystem {
    public abstract double getCentimeters();
}
