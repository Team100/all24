package org.team100.lib.motion.crank;

import org.team100.lib.profile.MotionProfile;

public class MotionProfileConstant implements MotionProfiles {
    private final MotionProfile m_profile;

    public MotionProfileConstant(MotionProfile profile) {
        m_profile = profile;
    }

    @Override
    public MotionProfile get() {
        return m_profile;
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }

}
