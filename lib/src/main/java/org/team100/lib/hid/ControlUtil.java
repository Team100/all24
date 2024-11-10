package org.team100.lib.hid;

import edu.wpi.first.math.MathUtil;

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

    public static double clamp(double input, double low, double high) {
        return MathUtil.clamp(input, low, high);
    }

    public static double scale(double raw, double negScale, double center, double posScale) {
        double zeroed = raw - center;
        if (zeroed < 0)
            return zeroed / negScale;
        return zeroed / posScale;
    }

    private ControlUtil() {
    }
}
