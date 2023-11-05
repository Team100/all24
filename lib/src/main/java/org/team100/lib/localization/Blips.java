package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;

/**
 * The entire payload from the camera. This is 1:1 copy of the representation in
 * the AprilTag library and our python wrapper.
 */
public class Blips {
    /**
     * Elapsed time of the analysis in python.
     */
    public final double et;

    /**
     * The set of targets seen by the camera.
     */
    public final List<Blip> tags;

    /**
     * For the deserializer.
     */
    protected Blips() {
        et = 0;
        tags = new ArrayList<Blip>();
    }

    @Override
    public String toString() {
        return "Blips [et=" + et + ", tags=" + tags + "]";
    }
}