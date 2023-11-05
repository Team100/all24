package frc.robot.armMotion;

import edu.wpi.first.math.geometry.Translation2d;

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
// -da.th1*l1 * Math.sin(a.th1) + -da.th2*l2 * Math.sin(a.th2) = dx
// da.th1*l1 * Math.cos(a.th1) + da.th2*l2 * Math.cos(a.th2) = dy
// da.th1 = (dx - da.th2*l2 * Math.cos(a.th2))/(l1 * Math.cos(a.th1))
// da.th2 = (dy - da.th1*l1 * Math.cos(a.th1))/()
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
     * @param x
     * @param y
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
    public ArmAngles inverseVel(ArmAngles pos, Translation2d vel) { 
        //Lower Arm Calculation
        double changeInLower = (Math.sin(pos.th2)*vel.getY() + Math.cos(pos.th2)*vel.getX())/(l1*Math.sin(pos.th2-pos.th1));
        //Upper Arm Calculation
        double changeInUpper = (Math.sin(pos.th1)*vel.getY() + Math.cos(pos.th1)*vel.getX())/(l2*Math.sin(pos.th1-pos.th2));
        ArmAngles dtheta = new ArmAngles(changeInLower, changeInUpper);
        return dtheta;
    }
}