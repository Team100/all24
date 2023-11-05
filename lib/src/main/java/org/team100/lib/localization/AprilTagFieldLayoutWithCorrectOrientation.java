package org.team100.lib.localization;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The WPILib JSON tag file, and the wrapper, AprilTagFieldLayout, define tag
 * rotation with respect to the *outward* normal, which is the opposite of the
 * Apriltags convention to use the *inward* normal.
 * 
 * This wrapper "fixes" the orientations so they match the Apriltag convention,
 * and thus match the result of camera pose estimates. Without this fix, we
 * would have to sprinkle inversions here and there, which would result in bugs.
 * 
 * @see https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#coordinate-system
 */
public class AprilTagFieldLayoutWithCorrectOrientation {
    // Inverts yaw
    private static final Transform3d kFix = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, Math.PI));
    private final AprilTagFieldLayout layout;

    // this is private because i don't want the red/blue enum in our code.
    /**
     * @param filename filename allows experimentation with other layouts
     */
    private AprilTagFieldLayoutWithCorrectOrientation(OriginPosition origin, String filename) throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(filename);
        layout = new AprilTagFieldLayout(path);
        layout.setOrigin(origin);
    }

    /**
     * @param filename allows experimentation with other layouts
     * @return Layout from the red perspective.
     */
    public static AprilTagFieldLayoutWithCorrectOrientation redLayout(String filename) throws IOException {
        return new AprilTagFieldLayoutWithCorrectOrientation(OriginPosition.kRedAllianceWallRightSide, filename);
    }

    /**
     * @param filename allows experimentation with other layouts
     * @return Layout from the blue perspective.
     */
    public static AprilTagFieldLayoutWithCorrectOrientation blueLayout(String filename) throws IOException {
        return new AprilTagFieldLayoutWithCorrectOrientation(OriginPosition.kBlueAllianceWallRightSide, filename);
    }

    /**
     * @return Tag pose with correct yaw (inverted compared to json file)
     */
    public Optional<Pose3d> getTagPose(int id) {
        Optional<Pose3d> pose = layout.getTagPose(id);
        if (!pose.isPresent()) {
            return pose;
        }
        return Optional.of(pose.get().transformBy(kFix));
    }
}
