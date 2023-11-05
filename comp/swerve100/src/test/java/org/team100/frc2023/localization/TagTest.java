package org.team100.frc2023.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Remind myself what's in the JSON file.
 * These are now inverted using the wrapper.
 */
public class TagTest {
    private static final double kDelta = 0.01;

    @Test
    public void testBlueLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-chargedup.json");
        /*
         * from the blue perspective, tag 5 has small x
         * and large y, and oriented at pi theta.
         */
        Pose3d tag5Pose = layout.getTagPose(5).get();
        assertEquals(0.36, tag5Pose.getTranslation().getX(), kDelta); // close to baseline
        assertEquals(6.75, tag5Pose.getTranslation().getY(), kDelta); // far to left
        assertEquals(0.69, tag5Pose.getTranslation().getZ(), kDelta); // 2 feet up
        assertEquals(0, tag5Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag5Pose.getRotation().getY(), kDelta);
        // "into the page" means facing towards the baseline, 180 degrees
        assertEquals(Math.PI, tag5Pose.getRotation().getZ(), kDelta);
    }

    @Test
    public void testRedLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-chargedup.json");
        /*
         * from the red perspective, tag 5 has large x
         * and small y, and oriented at zero theta.
         */
        Pose3d tag5Pose = layout.getTagPose(5).get();
        assertEquals(16.18, tag5Pose.getTranslation().getX(), kDelta); // far ahead
        assertEquals(1.26, tag5Pose.getTranslation().getY(), kDelta); // close to right side
        assertEquals(0.69, tag5Pose.getTranslation().getZ(), kDelta); // 2 feet up (as above)
        assertEquals(0, tag5Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag5Pose.getRotation().getY(), kDelta);
        // "into the page" i.e. away from the baseline, i.e. zero degrees
        assertEquals(0, tag5Pose.getRotation().getZ(), kDelta);
    }
}
