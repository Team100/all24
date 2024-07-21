package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SimulatedCamera;
import org.team100.lib.util.NotePicker;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Listen for updates from the note-detector camera and remember them for
 * awhile.
 */
public class NotePosition24ArrayListener {
    /** Ignore sights older than this. */
    private static final double kMaxSightAgeS = 0.1;
    private StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);
    private List<Translation2d> notes = new ArrayList<>();
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    private double latestTime = 0;

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

    public NotePosition24ArrayListener(SwerveDrivePoseEstimator100 poseEstimator) {
        m_poseEstimator = poseEstimator;
    }

    void consumeValues(NetworkTableEvent e) {
        ValueEventData ve = e.valueData;
        NetworkTableValue v = ve.value;
        String name = ve.getTopic().getName();
        String[] fields = name.split("/");
        if (fields.length != 3) {
            return;
        }
        if (fields[2].equals("fps")) {
            // FPS is not used by the robot
        } else if (fields[2].equals("latency")) {
            // latency is not used by the robot
        } else if (fields[2].equals("Rotation3d")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0) {
                return;
            }
            // NOTE! sights are x-ahead WPI coordinates, not z-ahead camera coordinates.
            Rotation3d[] sights;
            try {
                synchronized (m_buf) {
                    sights = m_buf.readArray(b);
                    latestTime = Timer.getFPGATimestamp();
                }
            } catch (RuntimeException ex) {
                return;
            }
            Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
            Pose2d robotPose = m_poseEstimator.getEstimatedPosition().pose();
            notes = TargetLocalizer.cameraRotsToFieldRelativeArray(
                    robotPose,
                    cameraInRobotCoordinates,
                    sights);
        } else {
            Util.warn("note weird vision update key: " + name);
        }
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTranslation2dArray() {
        Pose2d robotPose = m_poseEstimator.getEstimatedPosition().pose();
        switch (Identity.instance) {
            case BLANK:
                Transform3d simCameraInRobotCoordinates = Camera.GAME_PIECE.getOffset();
                SimulatedCamera simCamera = new SimulatedCamera(
                        simCameraInRobotCoordinates,
                        Math.toRadians(40),
                        Math.toRadians(31.5));
                List<Rotation3d> rot = simCamera.getRotation(
                        robotPose,
                        NotePicker.autoNotes);
                if (rot.isEmpty())
                    return new ArrayList<>();
                return TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        simCameraInRobotCoordinates,
                        rot.toArray(new Rotation3d[0]));
            default:
                if (latestTime > Timer.getFPGATimestamp() - kMaxSightAgeS) {
                    return notes;
                }
                return new ArrayList<>();
        }
    }

    /**
     * The field-relative translation of the closest note, if any.
     */
    public Optional<Translation2d> getClosestTranslation2d() {
        Pose2d robotPose = m_poseEstimator.getEstimatedPosition().pose();
        return NotePicker.closestNote(
                getTranslation2dArray(),
                robotPose);
    }

    public Optional<Translation2d> getTranslation2dAuto(Translation2d note) {
        return NotePicker.autoNotePick(getTranslation2dArray(), note);
    }

    public void enable() {
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        MultiSubscriber sub = new MultiSubscriber(inst, new String[] { "noteVision" },
                PubSubOption.keepDuplicates(true));
        inst.addListener(
                sub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}