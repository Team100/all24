package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

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
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Listen for updates from the note-detector camera and remember them for
 * awhile.
 * 
 * TODO: as described below, use a buffer here, don't just remember the very
 * last thing the camera saw. Also use multiple sights to get a better idea of
 * where the target is.
 */
public class NotePosition24ArrayListener {
    /** Ignore sights older than this. */
    private static final double kMaxSightAgeS = 0.1;
    private StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);
    private List<Translation2d> notes = new ArrayList<>();
    private final Supplier<Pose2d> m_poseSupplier;
    private final NetworkTableListenerPoller m_poller;

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

    public NotePosition24ArrayListener(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { "noteVision" },
                        PubSubOption.keepDuplicates(true)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
    }

    public void update() {
        for (NetworkTableEvent e : m_poller.readQueue()) {
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
                // TODO: this should use the timestamp of the camera data, not the current time.
                Pose2d robotPose = m_poseSupplier.get();
                // TODO: this should accumulate sights, not replace the list every time.
                notes = TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraInRobotCoordinates,
                        sights);
            } else {
                Util.warn("note weird vision update key: " + name);
            }
        }
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTranslation2dArray() {
        update();
        Pose2d robotPose = m_poseSupplier.get();
        switch (Identity.instance) {
            case BLANK:
                SimulatedCamera simCamera = SimulatedCamera.getGamePieceCamera();
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isEmpty())
                    return new ArrayList<>();
                List<Rotation3d> rot = simCamera.getKnownLocations(alliance.get(), robotPose);
                return TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        simCamera.getOffset(),
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
        update();
        Pose2d robotPose = m_poseSupplier.get();
        return NotePicker.closestNote(
                getTranslation2dArray(),
                robotPose);
    }
}