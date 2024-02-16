package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.CameraAngles;
import org.team100.lib.motion.drivetrain.Fixtured;

class TestCameraAngles extends Fixtured {
    @Test
    void testCameraAngles() {
        {
            CameraAngles camera = new CameraAngles(Math.toRadians(30), 1, 0,0);
               assertEquals(0,camera.getY(0,0),0.0001);
               assertEquals(-0.7,camera.getX(Math.toRadians(-25)),0.001);
               assertEquals(0.123465481082,camera.getY(Math.toRadians(-25),Math.toRadians(10)),0.001);
            //    assertEquals(416,camera.getInverseY(1,0),0.001);
            //    assertEquals(123.2,camera.getInverseX(1),0.001);
            }
    }
}
