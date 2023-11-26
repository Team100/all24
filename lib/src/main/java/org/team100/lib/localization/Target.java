package org.team100.lib.localization;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;

public class Target {

    /**
     * @return a target with the same pose as the specified tag, transformed
     */
    public static Optional<Pose2d> goal(
            AprilTagFieldLayoutWithCorrectOrientation layout,
            int tagID,
            Transform2d transform) {
        Optional<Pose3d> tagPose = layout.getTagPose(tagID);
        if (tagPose.isPresent()) {
            Pose2d m_tagPose = tagPose.get().toPose2d();
            return Optional.of(m_tagPose.plus(transform));
        }
        return Optional.empty();
    }
}
