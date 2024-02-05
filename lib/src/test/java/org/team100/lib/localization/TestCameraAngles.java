package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.CameraAngles;

class TestCameraAngles {
    @Test
    void testCameraAngles() {
        {
            CameraAngles camera = new CameraAngles(5, 67.5, 50, 832, 616, 1, new NotePosition24ArrayListener());
               assertEquals(0,camera.getX(416,200),0.0001);
               assertEquals(0.7,camera.getY(0),0.001);
        }
    }
}
