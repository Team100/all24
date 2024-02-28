package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedCameraTest {
    @Test
    void testNotePose() {
        SimulatedCamera simCamera = new SimulatedCamera(new Transform3d(new Translation3d(0,0,1),new Rotation3d(0,Math.toRadians(45),0)));
        Translation2d goal = new Translation2d(1,0.5);
        Translation2d[] array = {goal};
        ArrayList<Translation2d> arrayList = new ArrayList<>();
        arrayList.add(goal);
        Optional<ArrayList<Translation2d>> actual = simCamera.findNotes(new Pose2d(), array);
        System.out.println(actual);
        assertEquals(arrayList,actual.get());
    }
}
