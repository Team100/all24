package org.team100.lib.localization;

import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
 * in 2024 the pose returned from the camera has zero rotation when facing the
 * tag, which corresponds to the "inward normal" orientation.
 * 
 * the 2024 game map retains the "outward normal" orientation, and we're using
 * the WPILib wrapper around the Apriltag library, which does NOT invert the
 * canonical tag orientation.
 * 
 * AprilTagPoseEstimator.cpp seems to wrap the apriltag library, and then
 * transform the returned pose array into WPI the domain object with no
 * adjustment (i.e. it uses the raw rotation matrix, orthogonalized)
 * 
 * @see https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#coordinate-system
 * 
 */
public class AprilTagFieldLayoutWithCorrectOrientation {
    private static final String kProdFilename = "2024-crescendo.json";
    private static final String kPracticeFilename = "practice-field.json";

    // Inverts yaw
    private static final Transform3d kFix = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, Math.PI));

    private final Map<Alliance, AprilTagFieldLayout> layouts = new EnumMap<>(Alliance.class);
    private final Map<Alliance, AprilTagFieldLayout> practiceLayouts = new EnumMap<>(Alliance.class);

    public AprilTagFieldLayoutWithCorrectOrientation() throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(kProdFilename);

        AprilTagFieldLayout blueLayout = new AprilTagFieldLayout(path);
        blueLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        AprilTagFieldLayout redLayout = new AprilTagFieldLayout(path);
        redLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        layouts.put(Alliance.Red, redLayout);
        layouts.put(Alliance.Blue, blueLayout);

        Path practicePath = Filesystem.getDeployDirectory().toPath().resolve(kPracticeFilename);

        AprilTagFieldLayout bluePracticeLayout = new AprilTagFieldLayout(practicePath);
        bluePracticeLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        AprilTagFieldLayout redPracticeLayout = new AprilTagFieldLayout(practicePath);
        redPracticeLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        practiceLayouts.put(Alliance.Red, redPracticeLayout);
        practiceLayouts.put(Alliance.Blue, bluePracticeLayout);
    }

    /** Always use prod layouts. */
    public List<AprilTag> getTags(Alliance alliance) {
        return getLayout(alliance, 0).getTags();
    }

    /**
     * @return Tag pose with correct yaw (inverted compared to json file)
     */
    public Optional<Pose3d> getTagPose(Alliance alliance, int id) {
        AprilTagFieldLayout layout = getLayout(alliance, id);
        Optional<Pose3d> pose = layout.getTagPose(id);
        if (pose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(pose.get().transformBy(kFix));
    }

    private AprilTagFieldLayout getLayout(Alliance alliance, int id) {
        if (id >= 100) {
            return practiceLayouts.get(alliance);
        }
        return layouts.get(alliance);
    }
}
