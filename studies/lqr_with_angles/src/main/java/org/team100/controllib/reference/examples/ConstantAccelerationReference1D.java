package org.team100.controllib.reference.examples;

import edu.wpi.first.math.MathUtil;

public class ConstantAccelerationReference1D extends Reference1D {

    @Override
    public double position(double timeSec) {
        return MathUtil.angleModulus(Math.pow(timeSec, 2) / 2);
    }

    @Override
    public double velocity(double timeSec) {
        return timeSec;
    }

    @Override
    public double acceleration(double timeSec) {
        return 1.0;
    }

}
