package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Kinematics for two-jointed planar arm.
 */
public class ArmKinematics {
    private final double l1;
    private final double l2;

    /**
     * Lengths counting out from the grounded joint. Units here determine units
     * below.
     * 
     * @param l1 proximal
     * @param l2 distal
     */
    public ArmKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Calculates the position of the arm based on absolute joint angles, counting
     * out from the grounded joint.
     * 
     * @param a absolute angles
     * @return end position
     */
    public Translation2d forward(ArmAngles a) {
        return new Translation2d(
                l1 * Math.cos(a.th1) + l2 * Math.cos(a.th2),
                l1 * Math.sin(a.th1) + l2 * Math.sin(a.th2));
    }

    /**
     * Calculates the position of the elbow only, for visualization.
     */
    public Translation2d elbow(ArmAngles a) {
        return new Translation2d(l1 * Math.cos(a.th1), l1 * Math.sin(a.th1));
    }

    /**
     * Calculate absolute joint angles given cartesian coords of the end.
     * 
     * It's an application of the law of cosines. For diagram, see this doc:
     * https://docs.google.com/document/d/135U309CXN29X3Oube1N1DaXPHlo6r-YdnPHMH8NBev8/edit
     * 
     * @param t cartesian coordinate
     * @return absolute joint angles, null if unreachable.
     */
    public ArmAngles inverse(Translation2d t) {
        double r = Math.sqrt(t.getX() * t.getX() + t.getY() * t.getY());
        double gamma = Math.atan2(t.getY(), t.getX());
        double beta = Math.acos((r * r + l1 * l1 - l2 * l2) / (2 * r * l1));
        double alpha = Math.acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
        double th1 = gamma - beta;
        double th2 = Math.PI + th1 - alpha;
        if (Double.isNaN(th1) || Double.isNaN(th2))
            return null;
        return new ArmAngles(th1, th2);
    }

    /**
     * Calculate joint velocities given arm state and cartesian velocity
     * @param pos joint positions
     * @param vel cartesian velocity
     * @return joint velocities, rad/s
     */
    public ArmAngles inverseVel(ArmAngles pos, Translation2d vel) {
        if (pos == null) {
            return new ArmAngles(0, 0);
        }
        double dx = vel.getX();
        double dy = vel.getY();
        if (Math.abs(dx) < 0.001 && Math.abs(dy) < 0.001)
            return new ArmAngles(0, 0);

        if (Math.abs(pos.th1 - pos.th2) < 0.001) {
            // when th1 and th2 are the same, the arm is straight.
            // in that case, any movement along the arm requires infinite joint velocity
            return new ArmAngles(0, 0);
        }

        double dth1 = (dx * Math.cos(pos.th2) + dy * Math.sin(pos.th2))
                / (l1 * Math.sin(pos.th2 - pos.th1));

        double dth2 = (dx * Math.cos(pos.th1) + dy * Math.sin(pos.th1))
                / (l2 * Math.sin(pos.th1 - pos.th2));

        return new ArmAngles(dth1, dth2);
    }

}