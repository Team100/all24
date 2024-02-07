package org.team100.lib.localization;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;

/** For testing the blip24 struct array */
public class Blip24ArrayListener {

    StructBuffer<Blip24> m_buf = StructBuffer.create(Blip24.struct);

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
        } else if (fields[2].equals("blips")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0)
                return;
            Blip24[] blips;
            try {
                synchronized (m_buf) {
                    blips = m_buf.readArray(b);
                }
            } catch (RuntimeException ex) {
                return;
            }
            for (Blip24 blip : blips) {
                // this is where you would do something useful with the payload
                System.out.println(fields[1] + " " + blip);
            }
        } else {
            System.out.println("weird vision update key: " + name);
        }
    }

    public void enable() {
        NetworkTableInstance.getDefault().addListener(
                new String[] { "vision" },
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}
