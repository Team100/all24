package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;


public class Tapes {
    /**
     * The set of targets seen by the camera.
     */
    public final List<Tape> tapes;

    /**
     * For the deserializer.
     */
    protected Tapes() {
        tapes = new ArrayList<Tape>();
    }

    @Override
    public String toString() {
        return "";
    }
}