package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;
import org.team100.lib.util.NotePicker;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        switch (Identity.instance) {
            case BLANK:
                Optional<ArrayList<Translation2d>> optionalList = Optional.empty();
                ArrayList<Translation2d> list = new ArrayList<>();
                for (Translation2d note : NotePicker.autoNotes) {
                    Pose2d pose = new Pose2d(note, new Rotation2d());
                    Translation2d relative = pose.relativeTo(m_poseEstimator.getEstimatedPosition()).getTranslation();
                    Transform3d cameraInRobotCoordinates = Camera.get("10000000e31d4a24").getOffset();
                    double pitch;
                    double x = relative.getX() - cameraInRobotCoordinates.getX();
                    if (cameraInRobotCoordinates.getRotation().getZ() == Math.PI) {
                        pitch = Math.atan2(cameraInRobotCoordinates.getZ(), -1.0 * x)
                                - cameraInRobotCoordinates.getRotation().getY();
                    } else {
                        pitch = Math.atan2(cameraInRobotCoordinates.getZ(), x)
                                - cameraInRobotCoordinates.getRotation().getY();
                    }
                    double y = relative.getY() - cameraInRobotCoordinates.getY();
                    double yaw = MathUtil
                            .angleModulus(Math.atan2(y, x) - cameraInRobotCoordinates.getRotation().getZ());
                    Rotation3d rot = new Rotation3d(0, pitch, yaw);
                    if (Math.abs(pitch) < Math.toRadians(31.5) && Math.abs(yaw) < Math.toRadians(40)) {
                        Translation2d cameraRotationToRobotRelative = PoseEstimationHelper
                                .cameraRotationToRobotRelative(
                                        cameraInRobotCoordinates,
                                        rot);
                        Translation2d l = PoseEstimationHelper.convertToFieldRelative(
                                m_poseEstimator.getEstimatedPosition(),
                                cameraRotationToRobotRelative);
                        if (Math.abs(l.minus(note).getX()) < 0.01 && Math.abs(l.minus(note).getY()) < 0.01) {
                            list.add(l);
                            optionalList = Optional.of(list);
                        }
                    }
                }
                return optionalList;
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