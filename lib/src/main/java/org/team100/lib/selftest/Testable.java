package org.team100.lib.selftest;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.telemetry.Monitor;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.wpilibj2.command.Command;

/** This is for in-situ testing of stuff in RobotContainer. */
@ExcludeFromJacocoGeneratedReport
public interface Testable {
    SwerveDriveSubsystem getSwerveDriveSubsystem();
    Command getDrawCircle();
    Monitor getMonitor();
}
