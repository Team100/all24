package org.team100.two_joint_arm.subsystems.arm;

import org.team100.lib.config.Identity;

public class ArmFactory {

    private final Identity m_identity;

    public ArmFactory(Identity identity) {
        m_identity = identity;
    }

    public ArmInterface get() {
        switch (m_identity) {
            case COMP_BOT:
                return new ArmSubsystem();
            default:
                return new NoArmSubsystem();
        }
    }
}