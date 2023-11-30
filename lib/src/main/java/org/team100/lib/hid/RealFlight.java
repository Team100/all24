package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * The RealFlight USB controller is a basic RC-style control that comes with the
 * RealFlight simulator.
 * 
 * The control layout is as follows:
 * 
 * left x: axis 4
 * left y: axis 2 (with detents)
 * right x: axis 0
 * right y: axis 1
 * "reset/cancel": button 0
 * "menu/select": button 3
 * left switch: button 1
 * right switch: button 2
 * name: "Great Planes GP Controller"
 */
public class RealFlight implements Control {
    public static class Config {
        public double kDeadband = 0.02;
        public double kExpo = 0.5;
    }

    private final Config m_config = new Config();

    private final GenericHID hid;

    public RealFlight() {
        hid = new GenericHID(0);
    }

    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(hid.getRawAxis(0), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dy = expo(deadband(-1.0 * clamp(hid.getRawAxis(1), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(hid.getRawAxis(4), 1), m_config.kDeadband, 1), m_config.kExpo);
        return new Twist2d(dx, dy, dtheta);
    }

}
