package org.team100.lib.config;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceSelector {
    private final DigitalInput alliance1;
    private final DigitalInput alliance2;
    private final DriverStation.Alliance m_alliance;

    public AllianceSelector() {
        alliance1 = new DigitalInput(4);
        alliance2 = new DigitalInput(5);
        m_alliance = getAlliance();
    }

    public DriverStation.Alliance alliance() {
        return m_alliance;
    }

    public void close() {
        alliance1.close();
        alliance2.close();
    }

    /////////////////////////////////////////////

    // TODO: change the encoding so a valid result requires *some* bit to be set.
    // otherwise "disconnected" can be read as valid input.
    private DriverStation.Alliance getAlliance() {
        // remember these inputs are inverted
        int val = 0;
        if (alliance1.get())
            val += 1;
        if (alliance2.get())
            val += 2;
        // 0 is redAlliance
        if (val == 3) {
            return DriverStation.Alliance.Red;
        }
        return DriverStation.Alliance.Blue;
    }

}
