package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.PoseEstimationHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

class SimulatedCameraTest {
    @Test
    void testNotePose() {
        Transform3d cameraInRobotCoordinates = new Transform3d(new Translation3d(0,0,1),new Rotation3d(0,Math.toRadians(45),0));
        SimulatedCamera simCamera = new SimulatedCamera(cameraInRobotCoordinates, new Rotation3d(0,Math.toRadians(31.5),Math.toRadians(40)));
        Translation2d goal = new Translation2d(1,0); 
        Rotation3d rot = new Rotation3d();
        Translation2d[] array = {goal};
        ArrayList<Rotation3d> arrayList = new ArrayList<>();
        ArrayList<Translation2d> translationArrayList = new ArrayList<>();
        arrayList.add(rot);
        translationArrayList.add(goal);
        Pose2d currentPose = new Pose2d();
        Optional<ArrayList<Rotation3d>> actual = simCamera.getRotation(currentPose, array);
        assertEquals(arrayList,actual.get());
        assertEquals(PoseEstimationHelper.cameraRotsToFieldRelative(currentPose, cameraInRobotCoordinates,actual.get()),translationArrayList);
    }
}
