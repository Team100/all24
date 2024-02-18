package org.team100.lib.localization;

import java.util.EnumSet;
import java.util.Optional;
import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.util.CameraAngles;

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
    NotePosition24[] positions= new NotePosition24[0];
    private String id;
    double latestTime = 0;

    void consumeValues(NetworkTableEvent e) {
        ValueEventData ve = e.valueData;
        NetworkTableValue v = ve.value;
        String name = ve.getTopic().getName();
        String[] fields = name.split("/");
        id = fields[1];
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
        } else {
            System.out.println("note weird vision update key: " + name);
        }
    }

    private Optional<Double> optionalX(double position) {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                Optional<Double> e = Optional.of(position);
                return e;
            default:
                return Optional.empty();
        }
    }

    private Optional<Double> optionalY(double position) {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
                Optional<Double> e = Optional.of(position);
                return e;
            default:
                return Optional.empty();
        }
    }

    public Translation2d getTranslation2d() {
        switch (Identity.instance) {
            case BETA_BOT:
            case COMP_BOT:
            if (positions != null && latestTime > Timer.getFPGATimestamp()-0.1) {
                Transform3d cameraInRobotCoordinates = Camera.get(id).getOffset();
                CameraAngles e = new CameraAngles(cameraInRobotCoordinates);
                Double dx = null;
                Double dy = null;
                for (NotePosition24 note : positions) {
                    if (note.getPitch() < Math.PI / 2 - cameraInRobotCoordinates.getRotation().getY()) {
                    double s = note.getYaw();
                    dx = s;
                    double ss = note.getPitch();
                    dy = ss;
                    return e.getTranslation2d(new Rotation3d(0, optionalY(dy).get(), optionalX(dx).get()));
                    }
                }
                    return null;
                }
            default:
                return null;
        }
    }

    /**
     * @return The yaw to the object in the camera, 0 is center
     */
    public Optional<Double> getX() {
        double xd = getTranslation2d().getX();
        Optional<Double> e = Optional.of(xd);
        return e;
    }

    /**
     * @return The pitch to the object in the camera, 0 is center
     */
    public Optional<Double> getY() {
        double dy = getTranslation2d().getY();
        Optional<Double> e = Optional.of(dy);
        return e;
    }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "noteVision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
