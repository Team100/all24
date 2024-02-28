package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Optional;


import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SimulatedCamera;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.util.NotePicker;
import org.team100.lib.util.Util;

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

/** For testing the NotePosition struct array */
public class NotePosition24ArrayListener {
    private StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);
    private Optional<ArrayList<Translation2d>> notes = Optional.empty();
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    private double latestTime = 0;

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
            Rotation3d[] positions;
            try {
                synchronized (m_buf) {
                    positions = m_buf.readArray(b);
                    latestTime = Timer.getFPGATimestamp();
                }
            } catch (RuntimeException ex) {
                return;
            }
            Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
            notes = Optional.of(PoseEstimationHelper.cameraRotsToFieldRelativeArray(m_poseEstimator.getEstimatedPosition(),
            cameraInRobotCoordinates, positions));
        } else {
            Util.warn("note weird vision update key: " + name);
        }
    }

    /**
     * @return The translation of all the notes, field relative
     */
    public Optional<ArrayList<Translation2d>> getTranslation2dArray() {
        switch (Identity.instance) {
            case BLANK:
                Transform3d cameraInRobotCoordinates = Camera.GAME_PIECE.getOffset();
                SimulatedCamera simCamera = new SimulatedCamera(cameraInRobotCoordinates,
                        new Rotation3d(0, Math.toRadians(31.5), Math.toRadians(40)));
                Optional<ArrayList<Rotation3d>> rot = simCamera.getRotation(m_poseEstimator.getEstimatedPosition(),
                        NotePicker.autoNotes);
                    if (!rot.isPresent()) {
                        return Optional.empty();
                    }
                return Optional.of(PoseEstimationHelper.cameraRotsToFieldRelative(m_poseEstimator.getEstimatedPosition(),
                        cameraInRobotCoordinates, rot.get()));
            default:
                if (latestTime > Timer.getFPGATimestamp() - 0.1) {
                    return notes;
                }
                return Optional.empty();
        }
    }

    /**
     * @return The translation of all the closest note, field relative
     */
    public Optional<Translation2d> getClosestTranslation2d() {
        return NotePicker.closestNote(getTranslation2dArray(), m_poseEstimator.getEstimatedPosition());
    }

    public Optional<Translation2d> getTranslation2dAuto(int noteID) {
        return NotePicker.autoNotePick(getTranslation2dArray(), noteID);
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