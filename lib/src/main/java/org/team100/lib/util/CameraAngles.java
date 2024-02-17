package org.team100.lib.util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraAngles {
    private final double m_pitchRads;
    private final double m_yawRads;
    private final double m_cameraHeightMeters;
    private final double m_xOffset;
    private final double m_yOffset;

    /**
     * @param cameraInRobotCoordinates The transform3d of the camera in robot pose
     */
    public CameraAngles(
            Transform3d cameraInRobotCoordinates) {
        m_pitchRads = cameraInRobotCoordinates.getRotation().getY();
        m_yawRads = cameraInRobotCoordinates.getRotation().getZ();
        m_cameraHeightMeters = cameraInRobotCoordinates.getZ();
        m_xOffset = cameraInRobotCoordinates.getX();
        m_yOffset = cameraInRobotCoordinates.getY();
    }

    /**
     * 0 should be center of FOV
     * 
     * @return A robot relative translational x value of an object in a camera in
     *         meters
     */
    private double getX(double vertRotToTargetRads) {
        double x = m_cameraHeightMeters
                * Math.tan(vertRotToTargetRads + m_pitchRads);
        return x + m_xOffset;
    }

    /**
     * 0 should be center of FOV
     * 
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public Translation2d getTranslation2d(double vertRotToTargetRads, double horzRotToTargetRads) {
        double x = getX(vertRotToTargetRads);
        double y = x * Math.tan(horzRotToTargetRads + m_yawRads);
        return new Translation2d(x, y + m_yOffset);
    }
}