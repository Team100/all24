package org.team100.lib.localization;

import java.util.EnumSet;
import java.util.HashMap;
import org.team100.lib.config.Camera;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;

/** For testing the NotePosition struct array */
public class OpticalFlow24ArrayListener {
    private StructBuffer<Translation2d> m_buf = StructBuffer.create(Translation2d.struct);
    HashMap<Translation2d,Camera> list = new HashMap<>();
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    public OpticalFlow24ArrayListener(SwerveDrivePoseEstimator100 poseEstimator) {
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
        } else if (fields[2].equals("Translation2d")) {
            // decode the way StructArrayEntryImpl does
            byte[] b = v.getRaw();
            if (b.length == 0) {
                return;
            }
            Translation2d dthanslations;
            try {
                synchronized (m_buf) {
                    dthanslations = m_buf.read(b);
                }
            } catch (RuntimeException ex) {
                return;
            }
            Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
            updateOdometry(new Translation2d(cameraInRobotCoordinates.getZ()*Math.tan(dthanslations.getY()),cameraInRobotCoordinates.getZ()*Math.tan(dthanslations.getX())));
        } else {
            Util.warn("note weird vision update key: " + name);
        }
    }

    private void updateOdometry(Translation2d change) {
        Translation2d finalTranslation2d = change.plus(m_poseEstimator.getEstimatedPosition().translation());
    }

    public void enable() {
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        MultiSubscriber sub = new MultiSubscriber(inst, new String[] { "mouseVision" });
        inst.addListener(
                sub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                this::consumeValues);
    }
}