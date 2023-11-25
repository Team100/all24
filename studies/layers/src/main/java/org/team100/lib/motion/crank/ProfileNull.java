package org.team100.lib.motion.crank;

import org.team100.lib.profile.MotionProfile;

/** Supply nulls. */
public class ProfileNull implements MotionProfiles {

    @Override
    public MotionProfile get() {
        return null;
    }

    @Override
    public void accept(Indicator indicator) {
        indicator.indicate(this);
    }
}
