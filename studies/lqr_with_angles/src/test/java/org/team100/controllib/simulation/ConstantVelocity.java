package org.team100.controllib.simulation;

import org.team100.controllib.reference.Reference;
import org.team100.controllib.reference.examples.ConstantVelocityReference1D;

import edu.wpi.first.math.numbers.N2;

public class ConstantVelocity extends Scenario {
    private final Reference<N2> reference = new ConstantVelocityReference1D();

    Reference<N2> reference() {
        return reference;
    }

    String label() {
        return "CONSTANT VELOCITY";
    }

    //@Test
    public void test() {
        Loop loop = new Loop(this);
        loop.run();
    }
}