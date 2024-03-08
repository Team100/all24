package org.team100.lib.hid;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Checks periodically for changes in the HID connected to port 2, and changes
 * the third control implementation to match.
 */
public class ThirdControlProxy implements ThirdControl {
    private static class NoThirdControl implements ThirdControl {
    }
    private static final int kPort = 2;
    private static final double kFreq = 1;

    private final Notifier m_notifier;
    private String m_name;
    private ThirdControl m_ThirdControl;

    public ThirdControlProxy() {
        m_notifier = new Notifier(this::refresh);
        m_notifier.setName("ThirdControlProxy Notifier");
        refresh();
        m_notifier.startPeriodic(kFreq);
    }

    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        if (name.equals(m_name))
            return;
        m_name = name;
        m_ThirdControl = getThirdControl(name);

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("* Third HID: %s Control: %s *\n",
                m_ThirdControl.getHIDName(),
                m_ThirdControl.getClass().getSimpleName());
    }

    private static ThirdControl getThirdControl(String name) {
        if (name.contains("MIDI")) {
            return new ThirdMidiControl();
        }
        return new NoThirdControl();
    }

    @Override
    public String getHIDName() {
        return m_ThirdControl.getHIDName();
    }

    @Override
    public double shooterSpeed() {
        return m_ThirdControl.shooterSpeed();
    }
}
