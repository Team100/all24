package org.team100.lib.localization;

/**
 * Something the camera sees. This is 1:1 copy of the representation in the
 * AprilTag library.
 */
public class Tape {
    
    public final double[] pose_t;

    /**
     * Package-private for testing.
     */
    Tape(double[] pose_t) {
        this.pose_t = pose_t;
    }

    /**
     * For the deserializer.
     */
    protected Tape() {
        this.pose_t = new double[3];
    }

    @Override
    public String toString() {
        return "";
    }
}