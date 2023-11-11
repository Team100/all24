package org.team100.rolly_grabber.subsystems.manipulator;

import org.team100.lib.config.Identity;

public class ManipulatorFactory {

    private final Identity m_identity;

    public ManipulatorFactory(Identity identity) {
        m_identity = identity;
    }

    public ManipulatorInterface get() {
        switch (m_identity) {
            case COMP_BOT:
                return new Manipulator();
            default:
                return new NoManipulator();
        }
    }
}