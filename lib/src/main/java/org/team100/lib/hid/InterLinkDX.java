package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

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

    private final GenericHID hid;

    public InterLinkDX() {
        hid = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return hid.getName();
    }

    @Override
    public Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(scaled(4), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(scaled(3), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(scaled(0), 1), kDeadband, 1), kExpo);
        return new Velocity(dx, dy, dtheta);
    }

    @Override
    public boolean fullCycle() {
        return hid.getRawButton(13);
    }

    @Override
    public boolean resetRotation0() {
        return hid.getRawButton(14);
    }

    private double scaled(int axis) {
        double raw = hid.getRawAxis(axis);
        double zeroed = 0;
        switch (axis) {
            case 0:
                zeroed = -1 * raw - 0.000;
                System.out.println(zeroed);
                if (zeroed < 0)
                    return zeroed / 0.812;
                return zeroed / 0.850;
            case 1:
                zeroed = raw - 0.169;
                if (zeroed < 0)
                    return zeroed / 0.628;
                return zeroed / 0.619;
            case 2:
                zeroed = raw - 0.137;
                if (zeroed < 0)
                    return zeroed / 0.604;
                return zeroed / 0.643;

            case 3:
                zeroed =  raw + 0.008;
                if (zeroed < 0)
                    return zeroed / 0.859;
                return zeroed / 0.827;
            case 4:
                zeroed = -1 * raw - 0.031;
                if (zeroed < 0)
                    return zeroed / 0.836;
                return zeroed / 0.900;
            default:
                return 0;
        }
    }

}
