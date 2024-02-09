package org.team100.lib.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.ObjDoubleConsumer;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.lib.config.Camera;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Extracts robot pose estimates from camera input.
 * 
 * TODO: remove this class and the msgpack stuff.  See VisionDataProvider24.
 */
public class VisionDataProvider implements TableEventListener {
    /**
     * If the tag is closer than this threshold, then the camera's estimate of tag
     * rotation might be more accurate than the gyro, so we use the camera's
     * estimate of tag rotation to update the robot pose. If the tag is further away
     * than this, then the camera-derived rotation is probably less accurate than
     * the gyro, so we use the gyro instead.
     * 
     * Set this to zero to disable tag-derived rotation and always use the gyro.
     * 
     * Set this to some large number (e.g. 100) to disable gyro-derived rotation and
     * always use the camera.
     */
    private static final double kTagRotationBeliefThresholdMeters = 0.5;
    /** Discard results further than this from the previous one. */
    private static final double kVisionChangeToleranceMeters = 0.1;

    private final Telemetry t = Telemetry.get();

    private final Supplier<Pose2d> poseSupplier;
    private final ObjectMapper objectMapper;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final String m_name;

    // for blip filtering
    private Pose2d lastRobotInFieldCoords;

    public VisionDataProvider(
            AprilTagFieldLayoutWithCorrectOrientation layout,
            SwerveDrivePoseEstimator poseEstimator,
            Supplier<Pose2d> poseSupplier) throws IOException {
        // load the JNI (used by PoseEstimationHelper)
        CameraServerCvJNI.forceLoad();
        this.layout = layout;
        this.poseEstimator = poseEstimator;
        this.poseSupplier = poseSupplier;
        m_name = Names.name(this);

        objectMapper = new ObjectMapper(new MessagePackFactory());
    }

    /** Start listening for updates. */
    public void enable() {
        NetworkTable vision_table = NetworkTableInstance.getDefault().getTable("Vision");
        // Listen to ALL the updates in the vision table. :-)
        vision_table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), this);
    }

    /**
     * Accept a NetworkTableEvent and convert it to a Blips object
     * 
     * @param event the event to accept
     */
    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        try {
            Blips blips = objectMapper.readValue(event.valueData.value.getRaw(), Blips.class);
            estimateRobotPose(poseEstimator::addVisionMeasurement, key, blips);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * @param estimateConsumer is the pose estimator but exposing it here makes it
     *                         easier to test.
     * @param key              the camera identity, obtained from proc/cpuinfo
     * @param blips            all the targets the camera sees right now
     */
    void estimateRobotPose(
            ObjDoubleConsumer<Pose2d> estimateConsumer,
            String key,
            Blips blips) {
        for (Blip blip : blips.tags) {
            Optional<Pose3d> tagInFieldCordsOptional = layout.getTagPose(blip.id);
            if (!tagInFieldCordsOptional.isPresent())
                continue;

            Rotation2d gyroRotation = poseSupplier.get().getRotation();

            Transform3d cameraInRobotCoordinates = Camera.get(key).getOffset();

            // Gyro only produces yaw so use zero roll and zero pitch
            Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            Pose3d robotPoseInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                    cameraInRobotCoordinates,
                    tagInFieldCordsOptional.get(),
                    blip,
                    robotRotationInFieldCoordsFromGyro,
                    kTagRotationBeliefThresholdMeters);

            Translation2d robotTranslationInFieldCoords = robotPoseInFieldCoords.getTranslation().toTranslation2d();

            Pose2d currentRobotinFieldCoords = new Pose2d(robotTranslationInFieldCoords, gyroRotation);
            t.log(Level.DEBUG, m_name, "pose", currentRobotinFieldCoords);

            Rotation3d tagRotation = PoseEstimationHelper.blipToRotation(blip);
            t.log(Level.DEBUG, m_name, "Tag Rotation", tagRotation.getAngle());

            if (lastRobotInFieldCoords != null) {
                double distanceM = GeometryUtil.distance(lastRobotInFieldCoords,currentRobotinFieldCoords);
                if (distanceM <= kVisionChangeToleranceMeters) {
                    estimateConsumer.accept(currentRobotinFieldCoords, Timer.getFPGATimestamp() - .075);
                }
            }
            lastRobotInFieldCoords = currentRobotinFieldCoords;
        }
    }
}
