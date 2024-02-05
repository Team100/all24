package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.CameraAngles;

import edu.wpi.first.math.geometry.Pose2d;

class TestCameraAngles {
    @Test
    void testCameraAngles() {
        {
            Supplier<Pose2d> e = () -> new Pose2d();
            CameraAngles camera = new CameraAngles(5, 67.5, 50, 832, 616, 1, new NotePosition24ArrayListener(),e);
               assertEquals(0,camera.getY(416,200),0.0001);
               assertEquals(0.7,camera.getX(0),0.001);
        }
    }
}
