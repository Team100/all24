package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;

/**
 * Represents all the Raspberry Pi 4 cameras we have.
 */
public enum Camera {
    // these are in serial-number order
    // keep this synchronized with python tag_finder.py.
    A("1000000013c9c96c"),
    // B("100000004e0a1fb9"),
    // C("10000000a7c673d9"),
    B("10000000a7c673d9"),
    UNKNOWN(null);

    private static Map<String, Camera> cameras = new HashMap<>();
    static {
        for (Camera i : Camera.values()) {
            cameras.put(i.m_serialNumber, i);
        }
    }
    private String m_serialNumber;

    private Camera(String serialNumber) {
        m_serialNumber = serialNumber;
    }

    public static Camera get(String serialNumber) {
        if (cameras.containsKey(serialNumber))
            return cameras.get(serialNumber);
        return UNKNOWN;
    }
}
