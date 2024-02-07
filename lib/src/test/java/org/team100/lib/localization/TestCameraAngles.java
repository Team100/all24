package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.CameraAngles;
import org.team100.lib.motion.drivetrain.Fixtured;

class TestCameraAngles extends Fixtured {
    @Test
    void testCameraAngles() {
        {
            CameraAngles camera = new CameraAngles(30, 67.5, 50, 832, 616, 1, 0,0);
               assertEquals(0,camera.getY(416,200),0.0001);
               assertEquals(-0.7,camera.getX(0),0.001);
        }
    }
}
