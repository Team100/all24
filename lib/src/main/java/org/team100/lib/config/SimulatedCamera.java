package org.team100.lib.config;

import java.util.ArrayList;
import java.util.Optional;

import org.team100.lib.localization.PoseEstimationHelper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class SimulatedCamera {
    private final Transform3d m_cameraInRobotCoordinates;

    public SimulatedCamera(Transform3d cameraInRobotCoordinates) {
        m_cameraInRobotCoordinates = cameraInRobotCoordinates;
    }

    //TODO make this work with any camera yaw
    public Optional<ArrayList<Translation2d>> findNotes(Pose2d robotPose, Translation2d[] notes) {
        Optional<ArrayList<Translation2d>> optionalList = Optional.empty();
        ArrayList<Translation2d> list = new ArrayList<>();
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
            if (Math.abs(pitch) < Math.toRadians(31.5) && Math.abs(yaw) < Math.toRadians(40)) {
                Translation2d cameraRotationToRobotRelative = PoseEstimationHelper
                        .cameraRotationToRobotRelative(
                                m_cameraInRobotCoordinates,
                                rot);
                Translation2d l = PoseEstimationHelper.convertToFieldRelative(
                        robotPose,
                        cameraRotationToRobotRelative);
                if (Math.abs(l.minus(note).getX()) < 0.01 && Math.abs(l.minus(note).getY()) < 0.01) {
                    list.add(l);
                    optionalList = Optional.of(list);
                }
            }
        }
        return optionalList;
    }
}
