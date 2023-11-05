package org.team100.lib.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.lib.config.Cameras2023;
import org.team100.lib.telemetry.Telemetry;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Extracts robot pose estimates from camera input.
 */
public class VisionDataProvider extends Subsystem implements TableEventListener {
    public static class Config {
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
        public double kTagRotationBeliefThresholdMeters = 0.5;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final Supplier<Pose2d> poseSupplier;
    private final ObjectMapper objectMapper;
    private final SwerveDrivePoseEstimator poseEstimator;
    /** Discard results further than this from the previous one. */
    private final double kVisionChangeToleranceMeters = 0.1;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;

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
     * @param event the event to acceptx
     */
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        try {
            Blips blips = objectMapper.readValue(event.valueData.value.getRaw(), Blips.class);
            estimateRobotPose(Cameras2023::cameraOffset, poseEstimator::addVisionMeasurement, key, blips);
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
            Function<String, Transform3d> cameraOffsets,
            BiConsumer<Pose2d, Double> estimateConsumer,
            String key,
            Blips blips) {
        for (Blip blip : blips.tags) {
            Optional<Pose3d> tagInFieldCordsOptional = layout.getTagPose(blip.id);
            if (!tagInFieldCordsOptional.isPresent())
                continue;

            Rotation2d gyroRotation = poseSupplier.get().getRotation();

            Transform3d cameraInRobotCoordinates = cameraOffsets.apply(key);

            // Gyro only produces yaw so use zero roll and zero pitch
            Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            Pose3d robotPoseInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                    cameraInRobotCoordinates,
                    tagInFieldCordsOptional.get(),
                    blip,
                    robotRotationInFieldCoordsFromGyro,
                    m_config.kTagRotationBeliefThresholdMeters);

            Translation2d robotTranslationInFieldCoords = robotPoseInFieldCoords.getTranslation().toTranslation2d();

            Pose2d currentRobotinFieldCoords = new Pose2d(robotTranslationInFieldCoords, gyroRotation);
            t.log("/Vision Data Provider/Vision X", currentRobotinFieldCoords.getX());
            t.log("/Vision Data Provider/Vision Y", currentRobotinFieldCoords.getY());
            t.log("/Vision Data Provider/Vision Rotation", currentRobotinFieldCoords.getRotation().getRadians());

            Rotation3d tagRotation = PoseEstimationHelper.blipToRotation(blip);
            t.log("/Vision Data Provider/Tag Rotation", tagRotation.getAngle());

            if (lastRobotInFieldCoords != null) {
                Transform2d translationSinceLast = currentRobotinFieldCoords.minus(lastRobotInFieldCoords);
                double xComponent = translationSinceLast.getX();
                double yComponent = translationSinceLast.getY();
                if (xComponent * xComponent + yComponent * yComponent <= kVisionChangeToleranceMeters
                        * kVisionChangeToleranceMeters) {
                    estimateConsumer.accept(currentRobotinFieldCoords, Timer.getFPGATimestamp() - .075);
                }
            }
            lastRobotInFieldCoords = currentRobotinFieldCoords;
        }
    }
}
