package org.team100.lib.selftest;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public interface Testable {
    SwerveDriveSubsystem getSwerveDriveSubsystem();
    Command getDrawCircle();
    
}
