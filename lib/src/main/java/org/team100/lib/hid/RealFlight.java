package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.EnumLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.GenericHID;

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
 * Oscillate is "menu/select"
 * Left switch is medium speed.
 * Right switch is slow speed.
 */
public class RealFlight implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.15;

    private final GenericHID hid;
    private final EnumLogger m_log_speed;

    public RealFlight(SupplierLogger2 parent) {
        hid = new GenericHID(0);
        SupplierLogger2 child = parent.child(this);
        m_log_speed = child.enumLogger(Level.TRACE, "control_speed");
    }

    @Override
    public String getHIDName() {
        return hid.getName();
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    @Override
    public Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(scaled(0), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(scaled(1), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(scaled(4), 1), kDeadband, 1), kExpo);

        Speed speed = speed();
        m_log_speed.log(() -> speed);

        switch (speed) {
            case SLOW:
                return new Velocity(kSlow * dx, kSlow * dy, kSlow * dtheta);
            case MEDIUM:
                return new Velocity(kMedium * dx, kMedium * dy, kMedium * dtheta);
            default:
                return new Velocity(dx, dy, dtheta);
        }
    }

    @Override
    public boolean fullCycle() {
        return hid.getRawButton(4);
    }

    @Override
    public boolean resetRotation0() {
        return hid.getRawButton(1);
    }

    /////////////////////////////////////////

    private Speed speed() {
        // left
        if (hid.getRawButton(2))
            return Speed.SLOW;
        // right
        if (hid.getRawButton(3))
            return Speed.MEDIUM;
        return Speed.NORMAL;
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
