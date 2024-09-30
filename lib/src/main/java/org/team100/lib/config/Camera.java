package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents all the cameras. Some may be mounted on one robot, some on
 * another. Keep this up to date when you move cameras around.
 */
public enum Camera {

    C("10000000a7c673d9",
            new Transform3d(new Translation3d(0,0,1), new Rotation3d(0, -Math.toRadians(10), 0))),
    /**
     * Delta shooter
     */
    SHOOTER("10000000a7a892c0", 
            new Transform3d(
                    new Translation3d(-0.1265, 0.0682, 0.612),
                    new Rotation3d(0, Math.toRadians(-25), Math.toRadians(-2)))),

    /**
     * Delta amp-placer
     */
    RIGHTAMP("10000000caeaae82",
            new Transform3d(
                    new Translation3d(-0.1265, -0.1063625, 0.61),
                    new Rotation3d(0, Math.toRadians(-26), Math.toRadians(-63)))),

    /**
     * Delta amp-placer
     */
    LEFTAMP("100000004e0a1fb9",
            new Transform3d(
                    new Translation3d(-0.1265, 0.1532, 0.61),
                    new Rotation3d(0, Math.toRadians(-22), Math.toRadians(59)))),
    /**
     * Delta intake
     */
    GAME_PIECE("1000000013c9c96c",
            new Transform3d(
                new Translation3d(-0.1265, 0.03, 0.61),
                new Rotation3d(0, Math.toRadians(31.5), Math.PI))),

    /**
     * Camera bot intake
     */
    CAMERA_GAME_PIECE("364f07fb090a3bf7",
            new Transform3d(
                    new Translation3d(-0.0565, 0.03, 0.61),
                    new Rotation3d(0, Math.toRadians(30), Math.PI))),
                    
    TEST1("test1",
            new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, Math.PI / 4, 0))),

    TEST2("test2",
            new Transform3d(
                    new Translation3d(1, 0, 0),
                    new Rotation3d())),

    TEST3("test3",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, Math.PI/6, 0))),

    TEST4("test4",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, -Math.PI/4, 0))),

    UNKNOWN(null, new Transform3d());

    private static Map<String, Camera> cameras = new HashMap<>();
    static {
        for (Camera i : Camera.values()) {
            cameras.put(i.m_serialNumber, i);
        }
    }
    private String m_serialNumber;
    private Transform3d m_Offset;

    private Camera(String serialNumber, Transform3d offset) {
        m_serialNumber = serialNumber;
        m_Offset = offset;
    }

    public static Camera get(String serialNumber) {
        if (cameras.containsKey(serialNumber))
            return cameras.get(serialNumber);
        return UNKNOWN;
    }

    public Transform3d getOffset() {
        return m_Offset;
    }

    public String getSerial() {
        return m_serialNumber;
    }
}
