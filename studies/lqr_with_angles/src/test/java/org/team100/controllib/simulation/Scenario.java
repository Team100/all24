package org.team100.controllib.simulation;

import org.team100.controllib.reference.Reference;

import edu.wpi.first.math.numbers.N2;

public abstract class Scenario {

    abstract String label();

    abstract Reference<N2> reference();
}