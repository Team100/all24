package org.team100.lib.localization;

import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.config.NotePoseDetector;
import org.team100.lib.util.CameraAngles;

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
    NotePosition24[] positions;
    NotePoseDetector m_notePoseDetector;
    double latestTime = 0;

    public NotePosition24ArrayListener(NotePoseDetector notePoseDetector) {
        this.m_notePoseDetector = notePoseDetector;
    }

    void consumeValues(NetworkTableEvent e) {
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
        } else if (fields[2].equals("NotePosition24")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0)
                return;
            try {
                synchronized (m_buf) {
                    positions = m_buf.readArray(b);
                    latestTime = Timer.getFPGATimestamp();
                }
            } catch (RuntimeException ex) {
                return;
            }
            addVisionMeasurement(m_notePoseDetector::update, fields[1]);
        } else {
            System.out.println("note weird vision update key: " + name);
        }
    }

    /**
     * @return The x position in the camera in pixels, 0 should be the left of the
     *         screen
     */
    private Optional<Double> getX() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                double xd = positions[0].getX();
                Optional<Double> e = Optional.of(xd);
                return e;
            default:
                return Optional.empty();
        }
    }

    /**
     * @return The y position in the camera in pixels, 0 should be the bottom of the
     *         screen
     */
    private Optional<Double> getY() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                double dy = positions[0].getY();
                Optional<Double> e = Optional.of(dy);
                return e;
            default:
                return Optional.empty();
        }
    }

    private void addVisionMeasurement(Consumer<Translation2d> estimateConsumer, String ID) {
        Transform3d cameraInRobotCoordinates = Camera.get(ID).getOffset();
        CameraAngles e = new CameraAngles(cameraInRobotCoordinates);
        Translation2d d = e.getTranslation2d(getY().get(), getX().get());
        estimateConsumer.accept(d);
    }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "noteVision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
