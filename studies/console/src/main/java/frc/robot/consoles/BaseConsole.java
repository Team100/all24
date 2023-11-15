package frc.robot.consoles;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;

public class BaseConsole {
    private final int m_port;
    private int m_outputs; // access only with synchronize
    private int m_outputs_sent; // don't repeat yourself

    protected BaseConsole(int port) {
        m_port = port;
    }

    /*
     * Look up the port number using the Windows name -- each subconsole will
     * have a different name.
     */
    public static int portFromName(String name) {
        for (int i = 0; i < DriverStation.kJoystickPorts; ++i) {
            if (DriverStation.getJoystickName(i) == name) {
                return i;
            }
        }
        return -1;
    }

    /**
     * axis: 0-7
     */
    protected double getRawAxis(int axis) {
        if (m_port < 0)
            return 0;
        return DriverStation.getStickAxis(m_port, axis);
    }

    /**
     * button: 0-31
     */
    protected boolean getRawButton(int button) {
        if (m_port < 0)
            return false;
        return DriverStation.getStickButton(m_port, (byte) (button + 1));
    }

    protected void sendOutputs() {
        if (m_port < 0)
            return;
        synchronized (this) {
            if (m_outputs != m_outputs_sent) {
                DriverStationJNI.setJoystickOutputs((byte) m_port, m_outputs, (short) 0, (short) 0);
                m_outputs_sent = m_outputs;
            }
        }
    }

    /**
     * bit: 0-15
     */
    protected void setOutput(int bit, boolean value) {
        int val = value ? 1 : 0 << bit;
        int mask = ~(1 << bit);
        setOutput(val, mask);
    }

    /**
     * val: bits to write; zero elsewhere
     * mask: bits to keep, zero in the val field
     */
    protected void setOutput(int val, int mask) {
        synchronized (this) {
            m_outputs = (m_outputs & mask) | val;
        }
    }

    /**
     * Sets a field in the middle of the struct.
     */
    protected void applyOutput(int val, int width, int offset) {
        if (val > (1 << width) - 1) {
            return;
        }
        int mask = ~(((1 << width) - 1) << offset); // 0 where we want to write
        val = val << offset;
        setOutput(val, mask);
    }

    protected void setOutput(int bit) {
        setOutput(bit, true);
    }

    protected void clearOutput(int bit) {
        setOutput(bit, false);
    }
}
