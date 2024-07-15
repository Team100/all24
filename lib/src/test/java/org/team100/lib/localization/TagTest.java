package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Remind myself what's in the JSON file.
 * These are now inverted using the wrapper.
 */
class TagTest {
    private static final double kDelta = 0.01;

    @Test
    void testBlueLayout() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        /*
         * from the blue perspective, tag 7 has small x
         * and large y, and oriented at pi theta.
         */
        Pose3d tag7Pose = layout.getTagPose(Alliance.Blue, 7).get();
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
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        /*
         * from the red perspective, tag 7 has large x
         * and small y, and oriented at zero theta.
         */
        Pose3d tag7Pose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(16.5791, tag7Pose.getTranslation().getX(), kDelta); // far ahead
        assertEquals(2.663, tag7Pose.getTranslation().getY(), kDelta); // close to right side
        assertEquals(1.451, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m up (as above)
        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // "into the page" i.e. away from the baseline, i.e. zero degrees
        assertEquals(0, tag7Pose.getRotation().getZ(), kDelta);
    }

    @Test
    void testRaw() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("2024-crescendo.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        // blue side, tag seven
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        Pose3d tag7Pose = layout.getTagPose(7).get();

        // on our side, x is ~zero.
        assertEquals(-0.038, tag7Pose.getTranslation().getX(), kDelta); // behind the glass
        assertEquals(5.548, tag7Pose.getTranslation().getY(), kDelta); // far to left
        assertEquals(1.451, tag7Pose.getTranslation().getZ(), kDelta); // 1.5m feet up

        assertEquals(0, tag7Pose.getRotation().getX(), kDelta);
        assertEquals(0, tag7Pose.getRotation().getY(), kDelta);
        // raw rotation is "out of the page"
        assertEquals(0, tag7Pose.getRotation().getZ(), kDelta);
    }

    @Test
    void testPractice() throws IOException {
        {
            AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
            // from red perspective, 101 is far away
            Pose3d tag101Pose = layout.getTagPose(Alliance.Red, 101).get();
            assertEquals(8.0, tag101Pose.getTranslation().getX(), kDelta);
            assertEquals(2.0, tag101Pose.getTranslation().getY(), kDelta);
            assertEquals(1.0, tag101Pose.getTranslation().getZ(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getY(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getZ(), kDelta);
        }
        {
            AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
            Pose3d tag102Pose = layout.getTagPose(Alliance.Red, 102).get();
            assertEquals(0.0, tag102Pose.getTranslation().getX(), kDelta);
            assertEquals(2.0, tag102Pose.getTranslation().getY(), kDelta);
            assertEquals(1.0, tag102Pose.getTranslation().getZ(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getY(), kDelta);
            assertEquals(Math.PI, tag102Pose.getRotation().getZ(), kDelta);
        }
        {
            AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
            // from red perspective, 101 is near
            Pose3d tag101Pose = layout.getTagPose(Alliance.Blue, 101).get();
            assertEquals(0.0, tag101Pose.getTranslation().getX(), kDelta);
            assertEquals(2.0, tag101Pose.getTranslation().getY(), kDelta);
            assertEquals(1.0, tag101Pose.getTranslation().getZ(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getY(), kDelta);
            assertEquals(Math.PI, tag101Pose.getRotation().getZ(), kDelta);
        }
        {
            AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
            Pose3d tag102Pose = layout.getTagPose(Alliance.Blue, 102).get();
            assertEquals(8.0, tag102Pose.getTranslation().getX(), kDelta);
            assertEquals(2.0, tag102Pose.getTranslation().getY(), kDelta);
            assertEquals(1.0, tag102Pose.getTranslation().getZ(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getY(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getZ(), kDelta);
        }
    }

    @Test
    void testPracticeRaw() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve("practice-field.json");
        AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        {
            Pose3d tag101Pose = layout.getTagPose(101).get();

            // on our side, x is ~zero.
            assertEquals(0, tag101Pose.getTranslation().getX(), kDelta); // behind the glass
            assertEquals(2, tag101Pose.getTranslation().getY(), kDelta); // far to left
            assertEquals(1, tag101Pose.getTranslation().getZ(), kDelta); // 1.5m feet up

            assertEquals(0, tag101Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag101Pose.getRotation().getY(), kDelta);
            // raw rotation is "out of the page"
            assertEquals(0, tag101Pose.getRotation().getZ(), kDelta);
        }
        {
            Pose3d tag102Pose = layout.getTagPose(102).get();

            // on our side, x is ~zero.
            assertEquals(8, tag102Pose.getTranslation().getX(), kDelta); // behind the glass
            assertEquals(2, tag102Pose.getTranslation().getY(), kDelta); // far to left
            assertEquals(1, tag102Pose.getTranslation().getZ(), kDelta); // 1.5m feet up

            assertEquals(0, tag102Pose.getRotation().getX(), kDelta);
            assertEquals(0, tag102Pose.getRotation().getY(), kDelta);
            // raw rotation is "out of the page"
            assertEquals(Math.PI, tag102Pose.getRotation().getZ(), kDelta);
        }
    }
}
