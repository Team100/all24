package org.team100.lib.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** How the cameras were configured on the 2023 robot. */
public class Cameras2023 {

    /**
     * Camera views relative to the robot center at the floor.
     */
    public static Transform3d cameraOffset(String serialNumber) {

        Camera cam = Camera.get(serialNumber);

        switch (cam) {

            case B: // FRONT
                return new Transform3d(
                        new Translation3d(0, 0.2413, 0.3937),
                        new Rotation3d(0, 0, 0));
            case A: // LEFT
                return new Transform3d(
                        new Translation3d(0, 0.26035, 0.3937),
                        new Rotation3d(0, 0, 1.57));
            case UNKNOWN:
                return new Transform3d(
                        new Translation3d(0.255, 0.127, 0.3),
                        new Rotation3d(0, 0, 0));
            default:
                return new Transform3d();
        }
    }

    private Cameras2023() {}

}
