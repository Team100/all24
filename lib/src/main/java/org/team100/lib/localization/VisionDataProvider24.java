package org.team100.lib.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.ObjDoubleConsumer;

import org.team100.lib.config.Camera;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Extracts robot pose estimates from camera input.
 * 
 * This "24" version uses the "struct" method instead of the "msgpack" method,
 * which matches the TagFinder24 code on the camera.
 */
public class VisionDataProvider24 {
    /**
     * Time between events in reality and their appearance here; the average
     * end-to-end latency of the camera, detection code, network tables, and rio
     * looping.
     * 
     * Note this latency varies a little bit, depending on the relative timing of
     * the camera frame and the rio loop.
     * 
     * TODO: use the correct timing instead of this average.
     */
    private static final double kTotalLatencySeconds = 0.075;
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
    private static final double kTagRotationBeliefThresholdMeters = 0;
    /** Discard results further than this from the previous one. */
    private static final double kVisionChangeToleranceMeters = 0.1;

    private final Telemetry t = Telemetry.get();

    private final DoubleFunction<Optional<Rotation2d>> rotationSupplier;

    private final SwerveDrivePoseEstimator100 poseEstimator;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final String m_name;

    // for blip filtering
    private Pose2d lastRobotInFieldCoords;

    // reuse the buffer since it takes some time to make
    private StructBuffer<Blip24> m_buf = StructBuffer.create(Blip24.struct);

    /**
     * @param layout
     * @param poseEstimator
     * @param rotationSupplier rotation for the given time in seconds
     * @throws IOException
     */
    public VisionDataProvider24(
            AprilTagFieldLayoutWithCorrectOrientation layout,
            SwerveDrivePoseEstimator100 poseEstimator,
            DoubleFunction<Optional<Rotation2d>> rotationSupplier) throws IOException {
        // load the JNI (used by PoseEstimationHelper)
        CameraServerCvJNI.forceLoad();
        this.layout = layout;
        this.poseEstimator = poseEstimator;
        this.rotationSupplier = rotationSupplier;
        m_name = Names.name(this);
    }

    /** Start listening for updates. */
    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "vision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::accept);
    }

    /**
     * Accept a NetworkTableEvent and convert it to a Blips object
     * 
     * @param event the event to accept
     */

    
    public void accept(NetworkTableEvent e) {
        ValueEventData ve = e.valueData;
        NetworkTableValue v = ve.value;
        String name = ve.getTopic().getName();
        String[] fields = name.split("/");
        if (fields.length != 3)
            return;
        if (fields[2].equals("fps")) {
            // FPS is not used by the robot
        } else if (fields[2].equals("latency")) {
            // latency is not used by the robot
        } else if (fields[2].equals("blips")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0)
                return;
            Blip24[] blips;
            try {
                synchronized (m_buf) {
                    blips = m_buf.readArray(b);
                }
            } catch (RuntimeException ex) {
                return;
            }
            // the ID of the camera
            String cameraSerialNumber = fields[1];

            estimateRobotPose(
                    poseEstimator::addVisionMeasurement,
                    cameraSerialNumber,
                    blips);
        } else {
            // this event is not for us
            // Util.println("weird vision update key: " + name);
        }
    }

    /**
     * @param estimateConsumer   is the pose estimator but exposing it here makes it
     *                           easier to test.
     * @param cameraSerialNumber the camera identity, obtained from proc/cpuinfo
     * @param blips              all the targets the camera sees right now
     */
    void estimateRobotPose(
            ObjDoubleConsumer<Pose2d> estimateConsumer,
            String cameraSerialNumber,
            Blip24[] blips) {

        // Estimated instant represented by the blips
        double frameTime = Timer.getFPGATimestamp() - kTotalLatencySeconds;
        Optional<Rotation2d> optionalGyroRotation = rotationSupplier.apply(frameTime);

        if (optionalGyroRotation.isEmpty()) {
            Util.warn("No gyro rotation available!");
            return;
        }
        
        Rotation2d gyroRotation = optionalGyroRotation.get();

        // this treats every sight as independent.
        // TODO: cleverly combine sights with triangulation for more accuracy
        for (Blip24 blip : blips) {
            Optional<Pose3d> tagInFieldCordsOptional = layout.getTagPose(blip.getId());
            if (!tagInFieldCordsOptional.isPresent())
                continue;


            Transform3d cameraInRobotCoordinates = Camera.get(cameraSerialNumber).getOffset();

            // Gyro only produces yaw so use zero roll and zero pitch
            Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            t.log(Level.DEBUG, m_name, "Tag In Field Cords", tagInFieldCordsOptional.get().toPose2d());


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
                double distanceM = GeometryUtil.distance(lastRobotInFieldCoords, currentRobotinFieldCoords);
                if (distanceM <= kVisionChangeToleranceMeters) {
                    // this hard limit excludes false positives, which were a bigger problem in 2023
                    // due to the coarse tag family used. in 2024 this might not be an issue.
                    // TODO: WPI docs suggest update setVisionMeasurementStdDevs proportional to
                    // distance.
                    estimateConsumer.accept(currentRobotinFieldCoords, frameTime);
                }
            }
            lastRobotInFieldCoords = currentRobotinFieldCoords;
        }
    }
}
