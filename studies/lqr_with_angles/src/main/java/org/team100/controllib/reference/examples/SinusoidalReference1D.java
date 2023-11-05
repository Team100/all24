package org.team100.controllib.reference.examples;

import edu.wpi.first.math.MathUtil;

public class SinusoidalReference1D extends Reference1D {
    @Override
    public double position(double timeSec) {
        return MathUtil.angleModulus(Math.cos(timeSec));
    }

    @Override
    public double velocity(double timeSec) {
        return -1.0 * Math.sin(timeSec);
    }

    @Override
    public double acceleration(double timeSec) {
        return -1.0 * Math.cos(timeSec);
    }

}
