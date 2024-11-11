package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;
import static org.team100.lib.hid.ControlUtil.scale;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The Spektrum InterLinkDX controller is an RC-style control with a USB
 * interface.
 * 
 * This is unfinished.
 * 
 * TODO: finish the mapping
 * 
 * HARDWARE
 * 
 * The control layout is as follows:
 * 
 * left x: axis 0
 * left y: axis 1
 * left rear: axis 2
 * right x: axis 3
 * right y: axis 4
 * right rear: axis 5
 * R knob: axis 7
 * 
 * 
 * button 1: switch "A" 1 (0 is off)
 * button 2: switch "B" 0
 * button 3: switch "B" 2 (1 is off)
 * button 4: switch "C" 0
 * button 5: switch "C" 2 (1 is off)
 * button 6: switch "D" 0
 * button 7: switch "D" 2 (1 is off)
 * button 8: switch "F" 2
 * button 9: switch "F" 0 (1 is off)
 * button 10: switch "G" 2
 * button 11: switch "G" 0 (1 is off)
 * button 12: switch "H" 1 (0 is off)
 * button 13: switch "I"
 * button 14: switch "RESET"
 * button 15: switch "CANCEL"
 * button 16: switch "SELECT" push
 * button 17: switch "SELECT" to the left
 * button 18: switch "SELECT" to the right
 * button 19: left bottom trim left
 * button 20: left bottom trim right
 * button 21: left side trim down
 * button 22: left side trim up
 * button 23: right bottom trim left
 * button 24: right bottom trim right
 * button 25: right side trim down
 * button 26: right side trim up
 * button 27: ??
 */

public class InterLinkDX implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;
    private static final double kSlow = 0.25;

    private final GenericHID m_hid;

    public InterLinkDX() {
        m_hid = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return m_hid.getName();
    }

    @Override
    public Velocity velocity() {
        double dx = expo(deadband(
                clamp(scale(axis(4), 0.836, 0.031, 0.900), 1),
                kDeadband, 1),
                kExpo);
        double dy = expo(deadband(
                -1.0 * clamp(scale(axis(3), 0.859, -0.008, 0.827), 1),
                kDeadband, 1),
                kExpo);
        double dtheta = expo(deadband(
                -1.0 * clamp(scale(axis(0), 0.812, 0.0, 0.850), 1),
                kDeadband, 1),
                kExpo);
        if (button(1))
            return new Velocity(kSlow * dx, kSlow * dy, kSlow * dtheta);
        return new Velocity(dx, dy, dtheta);
    }

    @Override
    public boolean fullCycle() {
        return button(13);
    }

    @Override
    public boolean resetRotation0() {
        return button(14);
    }

    private double axis(int axis) {
        return m_hid.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_hid.getRawButton(button);
    }
}
