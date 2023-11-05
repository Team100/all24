package org.team100.lib.config;

import edu.wpi.first.wpilibj.DigitalInput;

public class AutonSelector {
    private final DigitalInput auto1;
    private final DigitalInput auto2;
    private final DigitalInput auto4;
    private final DigitalInput auto8;
    private final int m_routine;

    public AutonSelector() {
        auto1 = new DigitalInput(0);
        auto2 = new DigitalInput(1);
        auto4 = new DigitalInput(2);
        auto8 = new DigitalInput(3);
        m_routine = getAutoSwitchValue();
    }

    public int routine() {
        return m_routine;
    }

    public void close() {
        auto1.close();
        auto2.close();
        auto4.close();
        auto8.close();
    }

    /////////////////////////////////////////////
    
    public int getAutoSwitchValue() {
        int val = 0;
        if (auto8.get())
            val += 8;
        if (auto4.get())
            val += 4;
        if (auto2.get())
            val += 2;
        if (auto1.get())
            val += 1;
        return 15 - val;
    }

}
