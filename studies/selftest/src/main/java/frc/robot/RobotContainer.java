
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

/** Subsystems and commands can be tested in situ if they are package-private members. */
public class RobotContainer {
    final ExampleSubsystem subsystem;
    final Command command;

    public RobotContainer() {
        subsystem = new ExampleSubsystem();
        command = new ExampleCommand(subsystem);
    }
}
