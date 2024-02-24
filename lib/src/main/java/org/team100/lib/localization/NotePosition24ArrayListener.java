package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Optional;

import org.team100.lib.config.Camera;
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
            ArrayList<Translation2d> Tnotes = new ArrayList<>();
            Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
            for (Rotation3d position : positions) {
                if (position.getY() < cameraInRobotCoordinates.getRotation().getY()) {
                    Translation2d cameraRotationToRobotRelative = PoseEstimationHelper.cameraRotationToRobotRelative(
                            cameraInRobotCoordinates,
                            position);
                    Tnotes.add(PoseEstimationHelper.convertToFieldRelative(
                            m_poseEstimator.getEstimatedPosition(),
                            cameraRotationToRobotRelative));
                }
                // this is where you would do something useful with the payload
                // System.out.println(fields[1] + " " + position.getY() + " " +
                // position.getZ());
            }
            notes = Optional.of(Tnotes);
        } else {
            Util.warn("note weird vision update key: " + name);
        }
    }

    /**
     * @return The translation of all the notes, field relative
     */
    public Optional<ArrayList<Translation2d>> getTranslation2dArray() {
        if (latestTime > Timer.getFPGATimestamp() - 0.1) {
            return notes;
        }
        return Optional.empty();
    }

    /**
     * @return The translation of all the closest note, field relative
     */
    public Optional<Translation2d> getClosestTranslation2d() {
        if (latestTime > Timer.getFPGATimestamp() - 0.1) {
            return NotePicker.closestNote(notes, m_poseEstimator.getEstimatedPosition());
        }
        return Optional.empty();
    }

    public Optional<Translation2d> getTranslation2dAuto(int noteID) {
        if (latestTime > Timer.getFPGATimestamp() - 0.1) {
            return NotePicker.autoNotePick(notes, noteID);
        }
        return Optional.empty();
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