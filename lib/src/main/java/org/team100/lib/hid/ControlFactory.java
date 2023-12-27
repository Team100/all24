package org.team100.lib.hid;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Produces control instances based on their positions in the list and their
 * names. The driver control must be assigned to port zero. The operator control
 * must be assigned to port one. The constructor waits a few seconds before
 * giving up on disconnected ports.
 */
public class ControlFactory {
    private static final int kLimit = 5;
    private static final DriverControl kNoDriver = new DriverControl() {
    };
    private static final OperatorControl kNoOperator = new OperatorControl() {
    };
    private final DriverControl m_DriverControl;
    private final OperatorControl m_OperatorControl;

    public ControlFactory() {
        m_DriverControl = assignDriver();
        m_OperatorControl = assignOperator();

        Util.println("********************************** CONTROLS **********************************");
        Util.printf("*   Driver HID: %23.23s     Control: %23.23s *\n", m_DriverControl.getHIDName(),
                m_DriverControl.getClass().getSimpleName());
        Util.printf("* Operator HID: %23.23s     Control: %23.23s *\n", m_OperatorControl.getHIDName(),
                m_OperatorControl.getClass().getSimpleName());
        Util.println("******************************************************************************");

    }

    private DriverControl assignDriver() {
        GenericHID driverHID = new GenericHID(0);

        int waitCounter = 0;
        while (!driverHID.isConnected()) {
            if (waitCounter > kLimit) {
                return kNoDriver;
            }
            Util.println("Waiting for port zero...");
            sleep1();
            waitCounter += 1;
            DriverStation.refreshData();
        }

        String driverName = driverHID.getName();
        Util.println("Found driver HID: " + driverName);

        // TODO: add more xbox-like controls
        if (driverName.contains("F310")) {
            return new DriverXboxControl();
        }
        // TODO: make different classes for the different types of joysticks
        if (driverName.startsWith("VKBsim")
                || driverName.startsWith("Logitech Extreme")
                || driverName.contains("Sidewinder")) {
            return new JoystickControl();
        }
        if (driverName.startsWith("Great Planes")) {
            return new RealFlight();
        }
        if (driverName.equals("Team 100 Pilot")) {
            return new Pilot();
        }
        // TODO: make a class for simulation keyboard control
        if (driverName.contains("Keyboard")) {
            return new JoystickControl();
        }
        Util.println("Unrecognized driver control name: " + driverHID.getName());
        return kNoDriver;

    }

    private OperatorControl assignOperator() {
        GenericHID operatorHID = new GenericHID(1);

        int waitCounter = 0;
        while (!operatorHID.isConnected()) {
            if (waitCounter > kLimit) {
                return kNoOperator;
            }
            Util.println("Waiting for port one...");
            sleep1();
            waitCounter += 1;
            DriverStation.refreshData();
        }

        String operatorName = operatorHID.getName();
        Util.println("Found operator HID: " + operatorName);

        if (operatorName.contains("F310")) {
            return new OperatorXboxControl();
        }
        if (operatorName.startsWith("MSP430")) {
            // the old button board
            // TODO: make a class for it
            return kNoOperator;
        }
        // TODO: make a class for simulation keyboard control
        if (operatorName.contains("Keyboard")) {
            return new OperatorXboxControl();
        }
        Util.println("Unrecognized driver control name: " + operatorHID.getName());
        return kNoOperator;
    }

    private void sleep1() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
        }

    }

    public DriverControl getDriverControl() {
        return m_DriverControl;
    }

    public OperatorControl getOperatorControl() {
        return m_OperatorControl;
    }
}
