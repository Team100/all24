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
    /**
     * BETA Back
     */
    A("10000000caeaae82",
            new Transform3d(
                    new Translation3d(0.165, 0, 0.73),
                    new Rotation3d(0, -0.383972, Math.PI))),
    /**
     *BETA front
     */
    // B("1000000013c9c96c",
    //         new Transform3d(
    //                 new Translation3d(0, 0, 0.73),
    //                 new Rotation3d(0, -0.279253, 0))),
    B("1000000013c9c96c",
            new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0))),
    /**
     * Gamma intake
     */
    C("10000000a7c673d9",
            new Transform3d(
                    new Translation3d(0, 0.2413, 0.3937),
                    new Rotation3d(0, 0, 0))),
    /**
     * Delta shooter
     */
    D("10000000a7c673da",
            new Transform3d(
                    new Translation3d(0, 0.26035, 0.37465),
                    new Rotation3d(0, 0, 1.57))),
    /**
     * Delta amp-placer
     */
    E("10000000a7c673db",
            new Transform3d(
                    new Translation3d(0, 0.2413, 0.3937),
                    new Rotation3d(0, 0, 0))),
    /**
     * Delta intake
     */
    F("10000000a7c673dc",
            new Transform3d(
                    new Translation3d(0, 0.2413, 0.3937),
                    new Rotation3d(0, 0, 0))),

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
}
