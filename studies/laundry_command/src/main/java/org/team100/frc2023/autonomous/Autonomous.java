package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.LaundryArm;
import org.team100.frc2023.subsystems.LaundryDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {

    /** Dumps the cube and then drives for the specified speed and duration. */
    public Autonomous(LaundryDrive drive, double speed, double duration, LaundryArm arm) {
        addCommands(
                new Dump(arm),
                new Level(arm),
                new Drive(drive, speed, duration));
    }
}
