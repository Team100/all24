package org.team100.persistent_parameter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * the tunable parameter has four influences:
 * 1. the default
 * 2. the store
 * 3. the console
 * 4. the network
 * 
 * the logic is to use the store, if it exists,
 * and the default if not. the console provides
 * an offset, but the sum also needs to be written
 * to the store. the network can also provide updates.
 * 
 * so essentially we need to track
 * 
 */
public class ExampleSubsystem extends Subsystem {
    private static final double kDefault = 1.0;
    private static final String kKey = "foo";
    private final PersistentParameter p;

    /** @param knob console adjustment, absolute but starting at zero. */
    public ExampleSubsystem(DoubleSupplier knob) {
        // instantiated here because it has the key and the default
        p = new PersistentParameter(kKey, kDefault, knob);
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
