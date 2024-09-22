package org.team100.lib.localization;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.config.Camera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Publishes camera-relative tag poses to all cameras; this is for pan-and-zoom
 * in the camera.
 * 
 * TODO: move or filter this calculation?
 * The use of Transform3d (and inside, Quaternion) creates quite a bit of memory
 * pressure, so it might be better to do the transforms in the cameras, or to filter
 * just the tags that are in view.
 */
public class CameraUpdater {
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final Supplier<Pose2d> m_robotPose;
    private final List<Blip24> blips = new ArrayList<>();
    private HashMap<Camera, StructArrayPublisher<Blip24>> blipsPublishers = new HashMap<>();

    public CameraUpdater(Supplier<Pose2d> robotPose, AprilTagFieldLayoutWithCorrectOrientation layout)
            throws IOException {
        m_layout = layout;
        m_robotPose = robotPose;
        var inst = NetworkTableInstance.getDefault();
        // inst.startServer();
        Blip24Struct struct = new Blip24Struct();
        for (Camera camera : Camera.values()) {
            if (camera == Camera.UNKNOWN || camera == Camera.TEST1 || camera == Camera.TEST2 || camera == Camera.TEST3
                    || camera == Camera.TEST4)
                continue;
            blipsPublishers.put(camera, inst
                    .getStructArrayTopic("vision/" + camera.getSerial() + "/estimatedTagPose", struct).publish());
        }
        for (int i = 0; i < m_layout.getTags(Alliance.Red).size(); i++) {
            blips.add(new Blip24(i + 1, new Transform3d()));
        }
    }

    public void update() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            return;
        }
        for (Camera camera : Camera.values()) {
            if (camera == Camera.UNKNOWN || camera == Camera.TEST1 || camera == Camera.TEST2 || camera == Camera.TEST3
                    || camera == Camera.TEST4)
                return;
            for (int i = 1; i <= m_layout.getTags(alliance.get()).size(); i++) {
                Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance.get(), i);
                if (!tagInFieldCoordsOptional.isPresent())
                    return;
                Transform3d aprilTagTransform = PoseEstimationHelper.getTransformFromRobotPose(camera.getOffset(),
                        m_robotPose.get(), tagInFieldCoordsOptional.get());
                Blip24 blip = new Blip24(i, aprilTagTransform);
                blips.set(i - 1, blip);
                // System.out.println("Camera: " + camera);
                // System.out.println("ID: " + blip.getId());
                // System.out.println("Translation: "
                // + blip.getPose().getTranslation().toString());
                // System.out.println("X: " + blip.getPose().getRotation().getX() + " Y: " +
                // blip.getPose().getRotation().getY()
                // + " Z: " + blip.getPose().getRotation().getZ());
            }
            Blip24[] arrayList = new Blip24[m_layout.getTags(alliance.get()).size()];
            blips.toArray(arrayList);
            blipsPublishers.get(camera).set(arrayList);
        }
    }
}
