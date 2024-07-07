package org.team100.lib.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A simple practice field, 4 m wide, 8 m long, with goals at opposite ends and
 * a small obstacle in the center.
 * 
 * @see https://docs.google.com/document/d/11KpNoBe9oK7izn5RQoAD4uiy9C8vyJTv2SFmixqB-5g
 */
public class PracticeField {
    public static final double fieldLength = 8.0;
    public static final double fieldWidth = 4.0;

    // A one-foot circle in the center, like a traffic cone or a bucket.
    public static final Ellipse2d obstacle = new Ellipse2d(new Translation2d(4, 2), 0.3);

    public static final List<Ellipse2d> obstacles = List.of(obstacle);

    // Goal on the 0-meter baseline, with a tag under it.
    // Follows the "into the page" convention, so there is 180 degree rotation here.
    public static final Pose3d goa1 = new Pose3d(new Translation3d(0, 2, 1), new Rotation3d(0, 0, Math.PI));
    public static final Pose3d tag1 = new Pose3d(new Translation3d(0, 2, 1), new Rotation3d(0, 0, Math.PI));

    // Goal on the 8-meter baseline, with a tag under it.
    // Follows the "into the page" convention, so there is zero rotation here.
    public static final Pose3d goal2 = new Pose3d(new Translation3d(8, 2, 1), new Rotation3d());
    public static final Pose3d tag2 = new Pose3d(new Translation3d(8, 2, 1), new Rotation3d());

    public static final Map<Integer, Pose3d> tags = Map.of(
            1, tag1,
            2, tag2);

    private static final Pose3d blueOrigin = new Pose3d();
    private static final Pose3d redOrigin = new Pose3d(
            new Translation3d(fieldLength, fieldWidth, 0),
            new Rotation3d(0, 0, Math.PI));

    public static final Map<Alliance, Pose3d> origins = Map.of(
            Alliance.Blue, blueOrigin,
            Alliance.Red, redOrigin);

    public Optional<Pose3d> getTagPose(Alliance alliance, int id) {
        Pose3d pose = tags.get(id);
        if (pose == null)
            return Optional.empty();
        Pose3d origin = origins.get(alliance);
        if (origin == null)
            return Optional.empty();
        return Optional.of(pose.relativeTo(origin));
    }

    private PracticeField() {
        //
    }
}
