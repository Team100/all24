package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;

public interface MotionProfiles extends Supplier<MotionProfile>, Indicator.Visible {
}
