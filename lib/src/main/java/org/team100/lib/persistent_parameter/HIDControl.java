package org.team100.lib.persistent_parameter;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is really just an example.
 * 
 * The is for the HID called "Team 100 Knobs" which can be found in all24/studies/console/
 */
public class HIDControl {
    private static final double kScale = 10;
    private final CommandGenericHID m_hid;

    public HIDControl() {
        m_hid = new CommandGenericHID(0);
        Util.println("Found HID " + m_hid.getHID().getName());
    }

    /** Exposes knobs by id.  Knob id starts with zero. */
    public double knob(int id) {
        // The knob is mapped to axis 6 in the Arduino code.
        // For simulation use 0, which is mapped to "a" and "d"
        return kScale * m_hid.getRawAxis(id);
    }

    /** Exposes reset triggers.  Button id starts with one. */
    public Trigger reset(int id) {
        // the zero button is the "z" key in simulation
        return m_hid.button(id);
    }

    /** Button id starts with one. */
    public boolean button(int id) {
        return m_hid.getHID().getRawButton(id);
    }

}
