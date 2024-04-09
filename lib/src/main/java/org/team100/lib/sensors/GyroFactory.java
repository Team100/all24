package org.team100.lib.sensors;

import org.team100.lib.config.Identity;

public class GyroFactory {
    private final Identity m_identity;

    public GyroFactory(Identity identity) {
        m_identity = identity;
    }

    public Gyro100 get() {
        switch (m_identity) {
            case COMP_BOT:
            case SWERVE_ONE:
            case SWERVE_TWO:
            case BETA_BOT:
                return new SingleNavXGyro();
            default:
                return new NullGyro();
        }
    }
}
