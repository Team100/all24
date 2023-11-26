package org.team100.lib.telemetry;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Makes it easier to set the name of a SendableChooser.
 */
public class NamedChooser<T> extends SendableChooser<T> {
    private static final Set<String> names = new HashSet<>();

    /**
     * @param name must be globally unique.
     */
    public NamedChooser(String name) {
        super();
        if (names.contains(name))
            System.out.println("WARNING: duplicate named chooser: " + name);
        names.add(name);
        SendableRegistry.remove(this);
        SendableRegistry.add(this, name);
    }
}
