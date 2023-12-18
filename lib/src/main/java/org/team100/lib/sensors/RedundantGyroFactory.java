package org.team100.lib.sensors;

import org.team100.lib.config.Identity;

public class RedundantGyroFactory {
    private final Identity m_identity;

    public RedundantGyroFactory(Identity identity) {
        m_identity = identity;
    }

    public RedundantGyroInterface get() {
        switch (m_identity) {
            case COMP_BOT:
            case SWERVE_ONE:
            case SWERVE_TWO:
                return new RedundantGyro();
            default:
                return new NullRedundantGyro();
        }
    }
}
