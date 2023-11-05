package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Represents a type of HID controller.
 * 
 * These strings come from DriverStation.getJoystickName(), which concatenates
 * the Manufacturer and Product fields in the USB HID Device Descriptor.
 */
public enum Controller {
    LOGITECH_EXTREME_3D_PRO("Logitech Extreme 3D"),
    LOGITECH_F310("Controller (Gamepad F310)"),
    OLD_BUTTONS("MSP430-USB Gamepad"),
    PILOT("Team 100 Pilot"),
    SIDEWINDER_2("Microsoft 2 SideWinder Force Feedback 2 Joystick"),
    VKB_GLADIATOR("VKBsim Gladiator"),
    BLANK(""),
    UNKNOWN(null);

    private static Map<String, Controller> controllers = new HashMap<>();

    static {
        for (Controller i : Controller.values()) {
            controllers.put(i.m_name, i);
        }
    }

    private String m_name;

    private Controller(String name) {
        m_name = name;
    }

    public static Controller get(int stick) {
        String name = DriverStation.getJoystickName(stick);
        if (controllers.containsKey(name))
            return controllers.get(name);
        return UNKNOWN;
    }

    /*
     * For simulation only.
     */
    public static void set(int stick, Controller i) {
        DriverStationSim.setJoystickName(stick, i.m_name);
    }

}
