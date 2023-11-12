package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public RobotContainer() {
        CommandJoystick j = new CommandJoystick(0);
        ExampleSubsystem s = new ExampleSubsystem();
        // this is "z" in simulation
        j.trigger().onTrue(s.reset());
    }
}
