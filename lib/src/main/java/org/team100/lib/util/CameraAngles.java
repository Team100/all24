package org.team100.lib.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraAngles {
    private final Transform3d m_cameraInRobotCoordinates;

    /**
     * @param cameraInRobotCoordinates The transform3d of the camera in robot pose
     */
    public CameraAngles(
            Transform3d cameraInRobotCoordinates) {
        m_cameraInRobotCoordinates = cameraInRobotCoordinates;
    }

    /**
     * 0 should be center of FOV
     * 
     * @return A robot relative translational y value of an object in a camera in
     *         meters
     */
    public Translation2d getTranslation2d(Rotation3d cameraObject) {
        double x = m_cameraInRobotCoordinates.getZ() * Math.tan(cameraObject.getY() + Math.PI/2 - m_cameraInRobotCoordinates.getRotation().getY());
        double y = x * Math.tan(cameraObject.getZ() + m_cameraInRobotCoordinates.getRotation().getZ());
        return new Translation2d(x + m_cameraInRobotCoordinates.getX(), y + m_cameraInRobotCoordinates.getY());
    }
}