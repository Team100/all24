package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is really just an example.
 * See github.com/truher/console/arduino/pilot/Sensor.h
 */
public class HIDControl {
    private static final double kScale = 10;
    private final CommandGenericHID m_hid;

    public HIDControl() {
        m_hid = new CommandGenericHID(0);
    }

    /** Exposes knobs by id; doesn't know what they're for. */
    public double knob(int id) {
        // The knob is mapped to axis 6 in the Arduino code.
        // For simulation use 0, which is mapped to "a" and "d"
        return kScale * m_hid.getRawAxis(id);
    }

    /** This reset id is a button id which is one-based */
    public Trigger reset(int id) {
        // the zero button is the "z" key in simulation
        return m_hid.button(id);
    }

}
