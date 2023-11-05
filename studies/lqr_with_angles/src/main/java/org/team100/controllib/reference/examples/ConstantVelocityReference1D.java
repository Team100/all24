package org.team100.controllib.reference.examples;

import edu.wpi.first.math.MathUtil;

public class ConstantVelocityReference1D extends Reference1D {
    @Override
    public double position(double timeSec) {
        return MathUtil.angleModulus(timeSec);
    }

    @Override
    public double velocity(double timeSec) {
        return 1.0;
    }

    @Override
    public double acceleration(double timeSec) {
        return 0;
    }
}
