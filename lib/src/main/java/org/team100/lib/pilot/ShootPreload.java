package org.team100.lib.pilot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootPreload extends AutoPilot {

    private final Supplier<Pose2d> m_pose;

    public ShootPreload(Supplier<Pose2d> pose) {
        m_pose = pose;
    }

    /** Shoot from wherever you are. */
    @Override
    public Pose2d shootingLocation() {
        return m_pose.get();
    }

    @Override
    public boolean scoreSpeaker() {
        return enabled();
    }

}
