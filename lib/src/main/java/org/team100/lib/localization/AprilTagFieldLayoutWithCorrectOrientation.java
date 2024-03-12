package org.team100.lib.localization;

import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
 * 
 *      in 2024 the pose returned from the camera has zero rotation when facing
 *      the tag, which corresponds to the "inward normal" orientation.
 * 
 *      the 2024 game map retains the "outward normal" orientation, and we're
 *      using the WPILib wrapper
 *      around the Apriltag library, which does NOT invert the canonical tag
 *      orientation.
 * 
 *      AprilTagPoseEstimator.cpp seems to wrap the apriltag library, and then
 *      transform the
 *      returned pose array into WPI the domain object with no adjustment (i.e.
 *      it uses the raw
 *      rotation matrix, orthogonalized)
 */
public class AprilTagFieldLayoutWithCorrectOrientation {
    private static final String kFilename = "2024-crescendo.json";
   


      
    // Inverts yaw
    private static final Transform3d kFix = new Transform3d(
            new Translation3d(),
            new Rotation3d(0, 0, Math.PI));

    // private final AprilTagFieldLayout layout;

    private final Map<Alliance, AprilTagFieldLayout> layouts = new EnumMap<>(Alliance.class);


    /**
     * @param filename filename allows experimentation with other layouts
     */
    // private AprilTagFieldLayoutWithCorrectOrientation(OriginPosition origin, String filename) throws IOException {
    public AprilTagFieldLayoutWithCorrectOrientation() throws IOException {

        Path path = Filesystem.getDeployDirectory().toPath().resolve(kFilename);

        AprilTagFieldLayout blueLayout = new AprilTagFieldLayout(path);
        blueLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        AprilTagFieldLayout redLayout = new AprilTagFieldLayout(path);
        redLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        
        layouts.put(Alliance.Red, redLayout);
        layouts.put(Alliance.Blue, blueLayout);

    
        // layout = new AprilTagFieldLayout(path);
        // layout.setOrigin(origin);
    }

    // /**
    //  * @param filename allows experimentation with other layouts
    //  * @return Layout from the red perspective.
    //  */
    // public static AprilTagFieldLayoutWithCorrectOrientation redLayout(String filename) throws IOException {
    //     return new AprilTagFieldLayoutWithCorrectOrientation(OriginPosition.kRedAllianceWallRightSide, filename);
    // }

    // /**
    //  * @param filename allows experimentation with other layouts
    //  * @return Layout from the blue perspective.
    //  */
    // public static AprilTagFieldLayoutWithCorrectOrientation blueLayout(String filename) throws IOException {
    //     return new AprilTagFieldLayoutWithCorrectOrientation(OriginPosition.kBlueAllianceWallRightSide, filename);
    // }

    /**
     * @return Tag pose with correct yaw (inverted compared to json file)
     */
    public Optional<Pose3d> getTagPose(Alliance alliance, int id) {
        AprilTagFieldLayout layout = layouts.get(alliance);
        Optional<Pose3d> pose = layout.getTagPose(id);
        if (!pose.isPresent()) {
            return pose;
        }
        return Optional.of(pose.get().transformBy(kFix));
    }
}
