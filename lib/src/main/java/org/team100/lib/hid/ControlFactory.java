package org.team100.lib.hid;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Produces control instances based on their positions in the list and their
 * names
 * 
 * The driver control must be assigned to port zero!
 * 
 * The operator control must be assigned to port one!
 * 
 * The third control must be assigned to port two!
 * 
 * The constructor waits a few seconds before giving up on disconnected ports.
 */
public class ControlFactory {
    private static final int kLimit = 5;
    private static final DriverControl kNoDriver = new DriverControl() {
    };
    private static final OperatorControl kNoOperator = new OperatorControl() {
    };
    private static final ThirdControl kNoThird = new ThirdControl() {
    };
    private final DriverControl m_DriverControl;
    private final OperatorControl m_OperatorControl;
    // private final ThirdControl m_ThirdControl;

    public ControlFactory() {
        m_DriverControl = assignDriver();
        m_OperatorControl = assignOperator();
        // m_ThirdControl = assignThird();

        Util.println("********************************** CONTROLS **********************************");
        Util.printf("*   Driver HID: %23.23s     Control: %23.23s *\n", m_DriverControl.getHIDName(),
                m_DriverControl.getClass().getSimpleName());
        Util.printf("* Operator HID: %23.23s     Control: %23.23s *\n", m_OperatorControl.getHIDName(),
                m_OperatorControl.getClass().getSimpleName());
        // Util.printf("*    Third HID: %23.23s     Control: %23.23s *\n", m_ThirdControl.getHIDName(),
        //         m_ThirdControl.getClass().getSimpleName());
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

        if (driverName.contains("F310")) {
            return new DriverXboxControl();
        }
        if (driverName.startsWith("VKBsim")) {
            return new VKBJoystick();
        }
        if (driverName.startsWith("Logitech Extreme")) {
            return new LogitechExtremeJoystick();
        }
        if (driverName.startsWith("Great Planes")) {
            return new RealFlight();
        }
        if (driverName.equals("Team 100 Pilot")) {
            return new Pilot();
        }
        if (driverName.contains("Keyboard")) {
            return new SimulatedJoystick();
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
            return new OperatorV2Control();
        }
        if (operatorName.startsWith("MSP430")) {
            // the old button board
            return kNoOperator;
        }
        if (operatorName.contains("Keyboard")) {
            return new OperatorV2Control();
        }
        Util.println("Unrecognized driver control name: " + operatorHID.getName());
        return kNoOperator;
    }

    private ThirdControl assignThird() {
        GenericHID operatorHID = new GenericHID(2);
        int waitCounter = 0;
        while (!operatorHID.isConnected()) {
            if (waitCounter > kLimit) {
                return kNoThird;
            }
            Util.println("Waiting for port two...");
            sleep1();
            waitCounter += 1;
            DriverStation.refreshData();
        }

        String operatorName = operatorHID.getName();
        Util.println("Found operator HID: " + operatorName);

        if (operatorName.contains("MIDI")) {
            return new ThirdMidiControl();
        }
        Util.println("Unrecognized third control name: " + operatorHID.getName());
        return kNoThird;
    }

    private void sleep1() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            //
        }

    }

    public DriverControl getDriverControl() {
        return m_DriverControl;
    }

    public OperatorControl getOperatorControl() {
        return m_OperatorControl;
    }

    // public ThirdControl getThirdControl() {
    //     return m_ThirdControl;
    // }
}
