package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is really just an example.
 * See github.com/truher/console/arduino/pilot/Sensor.h
 */
public class HIDControl {
    private static final double kScale = 10;
    private final CommandJoystick m_joystick;

    public HIDControl() {
        m_joystick = new CommandJoystick(0);
    }

    public double getKnobValue() {
        // The knob is mapped to axis 6 in the Arduino code.
        // For simulation use 0, which is mapped to "a" and "d"
        return kScale * m_joystick.getRawAxis(0);
    }

    public Trigger reset() {
        // this is the "z" key in simulation
        return m_joystick.trigger();
    }

}
