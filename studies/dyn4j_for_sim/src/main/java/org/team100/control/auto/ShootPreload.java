package org.team100.control.auto;

import java.util.function.Supplier;

import org.team100.control.AutoPilot;

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
