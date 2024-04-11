package org.team100.lib.sensors;

import edu.wpi.first.math.geometry.Rotation3d;

public class EulerAngles {
    private final Rotation3d m_rotation;
    private final double m_accuracy;

    public EulerAngles(double roll, double pitch, double yaw, double accuracy) {
        m_rotation = new Rotation3d(roll, pitch, yaw);
        m_accuracy = accuracy;
    }

    public Rotation3d getRotation() {
        return m_rotation;
    }

    public double getAccuracy() {
        return m_accuracy;
    }
}
