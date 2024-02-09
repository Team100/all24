package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Remind myself what's in the JSON file.
 * These are now inverted using the wrapper.
 */
class TagTest {
    private static final double kDelta = 0.01;

    @Test
    void testBlueLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2024-crescendo.json");
        /*
         * from the blue perspective, tag 7 has small x
         * and large y, and oriented at pi theta.
         */
        Pose3d tag7Pose = layout.getTagPose(7).get();
        assertEquals(-0.038, tag7Pose.getTranslation().getX(), kDelta); // behind the glass
        assertEquals(5.548, tag7Pose.getTranslation().getY(), kDelta); // far to left
        assertEquals(1.451, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m feet up
        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // "into the page" means facing towards the baseline, 180 degrees
        assertEquals(Math.PI, tag7Pose.getRotation().getZ(), kDelta);
    }

    @Test
    void testRedLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2024-crescendo.json");
        /*
         * from the red perspective, tag 7 has large x
         * and small y, and oriented at zero theta.
         */
        Pose3d tag7Pose = layout.getTagPose(7).get();
        assertEquals(16.489, tag7Pose.getTranslation().getX(), kDelta); // far ahead
        assertEquals(2.663, tag7Pose.getTranslation().getY(), kDelta); // close to right side
        assertEquals(1.451, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m up (as above)
        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // "into the page" i.e. away from the baseline, i.e. zero degrees
        assertEquals(0, tag7Pose.getRotation().getZ(), kDelta);
    }
}
