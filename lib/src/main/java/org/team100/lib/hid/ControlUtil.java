package org.team100.lib.hid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;

public class ControlUtil {
    /**
     * Mix in the cube of the input; this feature is generally called "expo"
     * in the RC community even though it's not an exponential function.
     * 
     * @param input    range [-1,1]
     * @param fraction how much cubic to add, [0,1]
     */
    public static double expo(double input, double fraction) {
        return (1 - fraction) * input + fraction * input * input * input;
    }

    public static double deadband(double input, double threshold, double maxMagnitude) {
        return MathUtil.applyDeadband(input, threshold, maxMagnitude);
    }

    public static double clamp(double input, double clamp) {
        return MathUtil.clamp(input, -clamp, clamp);
    }

    public static Twist2d clampTwist(Twist2d input, double maxMagnitude) {
        double hyp = Math.hypot(input.dx, input.dy);
        double clamped = Math.min(hyp, maxMagnitude);
        double ratio = clamped / hyp;
        return new Twist2d(ratio * input.dx, ratio * input.dy, input.dtheta);
    }

    private ControlUtil() {
    }
}
