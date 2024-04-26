package org.team100.robot;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Stage post obstacles */
public class FieldMap {

    /**
     * Each post is a 30 cm square.
     */
    public static final double stagePostSizeM = 0.3;

    /**
     * These are from the OnShape CAD, adjusted a tiny bit to line up with the
     * background image in Glass.
     */
    public static final Map<String, Pose2d> stagePosts = Map.ofEntries(
            entry("east post", new Pose2d(3.38, 4.10, new Rotation2d(0))),
            entry("southeast post", new Pose2d(5.60, 2.80, new Rotation2d(-1))),
            entry("northeast post", new Pose2d(5.60, 5.38, new Rotation2d(1))),
            entry("southwest post", new Pose2d(10.95, 2.80, new Rotation2d(1))),
            entry("northwest post", new Pose2d(10.95, 5.38, new Rotation2d(-1))),
            entry("west post", new Pose2d(13.16, 4.10, new Rotation2d(0))));

    /**
     * These are approximate locations for the obstacle-avoidance force field, not
     * for collisions.
     */
    public static final Map<String, Pose2d> subwoofers = Map.ofEntries(
            entry("blue subwoofer", new Pose2d(0, 5.547, new Rotation2d(0))),
            entry("red subwoofer", new Pose2d(16.541, 5.547, new Rotation2d(0))));

    private FieldMap() {
        //
    }
}
