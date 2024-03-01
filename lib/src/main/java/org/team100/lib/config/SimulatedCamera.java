package org.team100.lib.config;

import java.util.ArrayList;
import java.util.Optional;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Creates a simulated camera with the given parameters, used for game piece detection testing
 */
public class SimulatedCamera {
    private final Transform3d m_cameraInRobotCoordinates;
    private final Rotation3d m_focalLength;

    /**
     * @param cameraInRobotCoordinates Parameters of the camera relative to the robot
     * @param focalLength Focal length of the camera, yaw is horizontal, pitch is vertical, should be in radians
     */
    public SimulatedCamera(Transform3d cameraInRobotCoordinates, Rotation3d focalLength) {
        m_cameraInRobotCoordinates = cameraInRobotCoordinates;
        m_focalLength = focalLength;
    }

    //TODO make this work with any camera yaw or roll
    /**
     * Gets the rotation to the object in the frame
     * @param robotPose Pose of the robot
     * @param notes field relative translation of any objects
     */
    public Optional<ArrayList<Rotation3d>> getRotation(Pose2d robotPose, Translation2d[] notes) {
        Optional<ArrayList<Rotation3d>> optionalList = Optional.empty();
        ArrayList<Rotation3d> list = new ArrayList<>();
        for (Translation2d note : notes) {
            Pose2d pose = new Pose2d(note, new Rotation2d());
            Translation2d relative = pose.relativeTo(robotPose).getTranslation();
            double pitch;
            double x = relative.getX() - m_cameraInRobotCoordinates.getX();
            if (m_cameraInRobotCoordinates.getRotation().getZ() == Math.PI) {
                pitch = Math.atan2(m_cameraInRobotCoordinates.getZ(), -1.0 * x)
                        - m_cameraInRobotCoordinates.getRotation().getY();
            } else {
                pitch = Math.atan2(m_cameraInRobotCoordinates.getZ(), x)
                        - m_cameraInRobotCoordinates.getRotation().getY();
            }
            double y = relative.getY() - m_cameraInRobotCoordinates.getY();
            double yaw = MathUtil
                    .angleModulus(Math.atan2(y, x) - m_cameraInRobotCoordinates.getRotation().getZ());
            Rotation3d rot = new Rotation3d(0, pitch, yaw);
            if (Math.abs(pitch) < m_focalLength.getY() && Math.abs(yaw) < m_focalLength.getZ()) {
                list.add(rot);
                optionalList = Optional.of(list);
            }
        }
        return optionalList;
    }
}
