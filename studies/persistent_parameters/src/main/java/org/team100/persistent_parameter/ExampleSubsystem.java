package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExampleSubsystem extends Subsystem {
    private final PersistentParameter p;

    public ExampleSubsystem() {
        // instantiated here because it has the key and the default
        p = new PersistentParameter("foo", 1.0);
    }

    @Override
    public void periodic() {
        System.out.println(p.get());
    }

    /** Only works when enabled. */
    public Command reset() {
        return runOnce(p::reset);
    }
    
}
