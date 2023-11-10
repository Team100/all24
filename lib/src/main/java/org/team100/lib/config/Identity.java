package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/*
 * Represents a specific RoboRIO, as a key for configurations.
 * 
 * The serial numbers here can be found on the label on the back: add a leading zero.
 */
public enum Identity {
    SWERVE_ONE("03238232"), // 0313baf3
    SWERVE_TWO("0317f285"),
    SQUAREBOT("031e31e3"),
    CAMERA_DOLLY("03126d76"),
    FRC_100_ea4("0306cea4"),
    TEST_BOARD_8D("03063c8d"),
    TEST_BOARD_B0("030628b0"),
    COMP_BOT("032363AC"),
    TEST_BOARD_6B("030d286b"),
    RIO_2022("foo"),
    TEAM100_2019("0317f0cd"),
    TEAM100_2018("0313baf3"),
    BLANK(""), // e.g. test default or simulation
    UNKNOWN(null);

    private static final Map<String, Identity> identities = new HashMap<>();
    private static String simIdentity = "";

    static {
        for (Identity i : Identity.values()) {
            identities.put(i.m_serialNumber, i);
        }
    }

    private final String m_serialNumber;

    private Identity(String serialNumber) {
        m_serialNumber = serialNumber;
    }

    public static Identity get() {
        String serialNumber = "";
        if (RobotBase.isReal()) {
            // Calling getSerialNumber in a vscode unit test
            // SEGVs because it does the wrong
            // thing with JNIs, so don't do that.
            serialNumber = RobotController.getSerialNumber();
        } else {
            serialNumber = simIdentity;
        }
        if (identities.containsKey(serialNumber))
            return identities.get(serialNumber);
        return UNKNOWN;
    }

    /*
     * For simulation and testing only.
     */
    static void set(String serialNumber) {
        if (!RobotBase.isReal()) {
            simIdentity = serialNumber;
        }
    }
}
