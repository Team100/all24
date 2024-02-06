package org.team100.lib.localization;

import java.util.EnumSet;
import java.util.Optional;

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
    double latestTime = 0;
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
            for (NotePosition24 position : positions) {
                // this is where you would do something useful with the payload
                // System.out.println(fields[1] + " " + position);
            }
        } else {
            System.out.println("note weird vision update key: " + name);
        }
    }

    /**
     * @return The x position in the camera in pixels, 0 should be the left of the screen
    */
    public Optional<Double> getX() {
            double xd = positions[0].getX();
            Double x = xd;
            Optional<Double> e = Optional.of(x);
            return e;
    }

    /**
     * @return The y position in the camera in pixels, 0 should be the bottom of the screen 
    */
    public Optional<Double> getY() {
            double dy = positions[0].getY();
            Double y = -1.0 * (dy-616);
            Optional<Double> e = Optional.of(y);
            return e;
        }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "vision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
