package org.team100.lib.localization;

import java.util.EnumSet;

import org.opencv.core.Point;
import org.team100.lib.util.CameraAngles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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

    public Double getX() {
        if (latestTime < Timer.getFPGATimestamp()-0.1) {
            return null;
        }
        if (positions != null) {
            double x = positions[0].getX();
            return (x-416)/416;
        }
        return null;
    }

    public Double getY() {
        if (latestTime < Timer.getFPGATimestamp()-0.1) {
            return null;
        }
        if (positions != null) {
            double y = positions[0].getY();
            return (y-308)/308;
        }
        return null;
    }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "vision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
