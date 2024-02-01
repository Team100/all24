package org.team100.lib.hid;

import org.team100.frc2024.RobotContainer;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Checks periodically for changes in the HID connected to port 0, and changes
 * the driver control implementation to match.
 */
public class DriverControlProxy implements DriverControl {
    private static class NoDriverControl implements DriverControl {
    }
    private static final int kPort = 0;
    private static final double kFreq = 1;

    private final Notifier m_notifier;
    private String m_name;
    private DriverControl m_driverControl;
    private RobotContainer m_container;

    public DriverControlProxy(RobotContainer container) {
        m_notifier = new Notifier(this::refresh);
        refresh();
        m_container = container;

        m_notifier.startPeriodic(kFreq);
    }


    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        if (name.equals(m_name))
            return;
        m_name = name;
        m_driverControl = getDriverControl(name);

        //It dosent run for this first time because it dosent need to rebind during the first cycle
        if(m_container != null){
            m_container.bindDriverControls();
        }

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("*** Driver HID: %s Control: %s\n",
                m_driverControl.getHIDName(),
                m_driverControl.getClass().getSimpleName());
    }

    private static DriverControl getDriverControl(String name) {
        if (name.contains("F310")) {
            return new DriverXboxControl();
        }
        if (name.startsWith("VKBsim")) {
            return new VKBJoystick();
        }
        if (name.startsWith("Logitech Extreme")) {
            return new LogitechExtremeJoystick();
        }
        if (name.startsWith("Great Planes")) {
            return new RealFlight();
        }
        if (name.equals("Team 100 Pilot")) {
            return new Pilot();
        }
        if (name.contains("Keyboard")) {
            return new SimulatedJoystick();
        }
        return new NoDriverControl();
    }

    @Override
    public String getHIDName() {
        return m_driverControl.getHIDName();
    }

    @Override
    public Twist2d twist() {
        return m_driverControl.twist();
    }

    @Override
    public Rotation2d desiredRotation() {
        return m_driverControl.desiredRotation();
    }

    @Override
    public Translation2d target() {
        return m_driverControl.target();
    }

    @Override
    public boolean trigger() {
        return m_driverControl.trigger();
    }

    @Override
    public Trigger resetPose() {
        return m_driverControl.resetPose();
    }

    @Override
    public Trigger resetRotation0() {
        return m_driverControl.resetRotation0();
    }

    @Override
    public Trigger resetRotation180() {
        return m_driverControl.resetRotation180();
    }

    @Override
    public Trigger driveSlow() {
        return m_driverControl.driveSlow();
    }

    @Override
    public Trigger driveMedium() {
        return m_driverControl.driveMedium();
    }

    @Override
    public Speed speed() {
        return m_driverControl.speed();
    }

    @Override
    public Trigger defense() {
        return m_driverControl.defense();
    }

    @Override
    public Trigger steer0() {
        return m_driverControl.steer0();
    }

    @Override
    public Trigger steer90() {
        return m_driverControl.steer90();
    }

    @Override
    public Trigger rotate0() {
        return m_driverControl.rotate0();
    }

    @Override
    public Trigger driveWithFancyTrajec() {
        return m_driverControl.driveWithFancyTrajec();
    }

    @Override
    public Trigger circle() {
        return m_driverControl.circle();
    }

    @Override
    public Trigger actualCircle() {
        return m_driverControl.actualCircle();
    }

    @Override
    public Trigger never() {
        return m_driverControl.never();
    }

    @Override
    public boolean annunicatorTest() {
        return m_driverControl.annunicatorTest();
    }

    @Override
    public Trigger test(){
        return m_driverControl.test();
    }
}
