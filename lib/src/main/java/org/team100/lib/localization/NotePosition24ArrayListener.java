package org.team100.lib.localization;

import java.util.EnumSet;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.copies.SwerveDrivePoseEstimator100;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.Timer;

/** For testing the NotePosition struct array */
public class NotePosition24ArrayListener {
    StructBuffer<NotePosition24> m_buf = StructBuffer.create(NotePosition24.struct);
    Optional<Pose2d>[] notes = new Optional[] { Optional.empty() };
    SwerveDrivePoseEstimator100 m_poseEstimator;
    Optional<Translation2d>[] Tnotes = new Optional[] { Optional.empty() };
    double latestTime = 0;

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
        } else if (fields[2].equals("NotePosition24")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0)
                return;
            NotePosition24[] positions;
            try {
                synchronized (m_buf) {
                    positions = m_buf.readArray(b);
                    latestTime = Timer.getFPGATimestamp();
                }
            } catch (RuntimeException ex) {
                return;
            }
            notes = new Optional[] { Optional.empty() };
            int noteNum = 0;
            Optional<Double> y = Optional.empty();
            Optional<Double> x = Optional.empty();
            Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
            for (NotePosition24 position : positions) {
                if (position.getPitch() < Math.PI / 2 - cameraInRobotCoordinates.getRotation().getY()) {
                    double dy = position.getPitch();
                    Double yz = dy;
                    y = Optional.of(yz);
                    double xd = position.getYaw();
                    Double dv = xd;
                    x = Optional.of(dv);
                    notes[noteNum] = Optional
                            .of(PoseEstimationHelper.convertToFieldRelative(m_poseEstimator.getEstimatedPosition(),
                                    PoseEstimationHelper.cameraRotationToRobotRelative(cameraInRobotCoordinates,
                                            new Rotation3d(0, y.get(), x.get()))));
                    Tnotes[noteNum] = Optional.of(notes[noteNum].get().getTranslation());
                }
                // this is where you would do something useful with the payload
                // System.out.println(fields[1] + " " + position);
            }
        } else {
            System.out.println("note weird vision update key: " + name);
        }
    }

    /**
     * @return The translation of the note, field relative, rotation can be ignored,
     *         as it is field relative rotation from robot to note
     */
    public Optional<Pose2d>[] getPose2d() {
        return notes;
    }

    public Optional<Translation2d>[] getTranslation2d() {
        return Tnotes;
    }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "noteVision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
