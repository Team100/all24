package org.team100.lib.planner;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

public interface RobotModel<States extends Num> {

    /** The initial state. */
    Matrix<States, N1> initial();

    /** The goal state. */
    Matrix<States, N1> goal();

    /** True if the configuration is in the goal region */
    boolean goal(Matrix<States, N1> state);

    /** True if the config is not within an obstacle. */
    boolean clear(Matrix<States, N1> state);

    /** True if the link is feasible. */
    boolean link(Matrix<States, N1> source, Matrix<States, N1> target);
}
