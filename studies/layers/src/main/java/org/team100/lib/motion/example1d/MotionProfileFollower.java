package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.framework.SpecFollower;
import org.team100.lib.motion.example1d.framework.Workstate;
import org.team100.lib.profile.MotionProfile;

/** 1d profile follower */
public class MotionProfileFollower implements SpecFollower<Workstate<Double>> {

    MotionProfile m_profile;

    public MotionProfileFollower(MotionProfile profile) {
        m_profile = profile;
    }

    // TODO : this is wrong, needs to return the whole state not just X
    @Override
    public Workstate<Double> getReference(double time) {
        return new CrankWorkstate(m_profile.get(time).getX());
    }

}
