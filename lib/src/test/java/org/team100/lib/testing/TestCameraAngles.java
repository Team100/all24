package org.team100.lib.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.CameraAngles;

class TrapezoidProfile100Test {

    /** Now we expose acceleration in the profile state, so make sure it's right. */
    @Test
    void testCameraAngles() {
        {
            CameraAngles camera = new CameraAngles(5, 67.5, 50, 832, 616, 1);
               assertEquals(0,camera.getX(416,200),0.0001);
        }
    }
}
