package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.clampTwist;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The RealFlight USB controller is a basic RC-style control that comes with the
 * RealFlight simulator.
 * 
 * HARDWARE
 * 
 * The control layout is as follows:
 * 
 * left x: axis 4
 * left y: axis 2 (with detents)
 * right x: axis 0
 * right y: axis 1
 * remember buttons are one-based
 * "reset/cancel": button 1
 * "menu/select": button 4
 * left switch: button 2
 * right switch: button 3
 * name: "Great Planes GP Controller"
 * 
 * MAPPINGS
 * 
 * Right x and y are mapped to cartesian control.
 * Left x is rotational speed
 * Left y is throttle, slow-med-fast.
 * Reset is "reset pose"
 * Left switch is medium speed.
 * Right switch is slow speed.
 */
public class RealFlight implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final CommandGenericHID hid;

    public RealFlight() {
        hid = new CommandGenericHID(0);
    }

    @Override
    public String getHIDName() {
        return hid.getHID().getName();
    }

    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(scaled(0), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(scaled(1), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(scaled(4), 1), kDeadband, 1), kExpo);
        return clampTwist(new Twist2d(dx, dy, dtheta), 1.0);
    }

    @Override
    public Trigger resetPose() {
        return hid.button(1);
    }

    /**
     * Use slow when the right switch is on
     */
    @Override
    public Trigger driveSlow() {
        return rightSwitchOn();
    }

    /**
     * Use medium when the left switch is on but the right one is not
     * (slow overrides medium)
     */
    @Override
    public Trigger driveMedium() {
        Trigger rightSwitchOff = rightSwitchOn().negate();
        return leftSwitchOn().and(rightSwitchOff);
    }

    @Override
    public Speed speed() {
        // left
        if (hid.getHID().getRawButton(2))
            return Speed.SLOW;
        // right
        if (hid.getHID().getRawButton(3))
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    /////////////////////////////////////////

    public Trigger leftSwitchOn() {
        return hid.button(2);
    }

    public Trigger rightSwitchOn() {
        return hid.button(3);
    }

    /**
     * Scale to [-1,1] with the center at 0.
     * Each axis calibration is different.
     */
    private double scaled(int axis) {
        double raw = hid.getRawAxis(axis);
        double zeroed = 0;
        switch (axis) {
            case 0:
                zeroed = raw - 0.043;
                if (zeroed < 0)
                    return zeroed / 0.729;
                return zeroed / 0.784;
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
                return 0;
            case 4:
                zeroed = raw - 0.075;
                if (zeroed < 0)
                    return zeroed / 0.738;
                return zeroed / 0.776;
            default:
                return 0;
        }
    }
}
