package org.team100.lib.motion.arm;

/**
 * Represents a 2DOF serial arm.
 */
public class ArmAngles {
    /** absolute proximal radians */
    public final double th1;
    /** absolute distal radians */
    public final double th2;

    /**
     * Absolute angles in radians, counting out from the grounded joint.
     * 
     * @param th1 proximal
     * @param th2 distal
     */
    public ArmAngles(double th1, double th2) {
        this.th1 = th1;
        this.th2 = th2;
    }
}