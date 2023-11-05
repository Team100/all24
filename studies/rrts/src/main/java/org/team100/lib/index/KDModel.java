package org.team100.lib.index;

import org.team100.lib.graph.Node;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

public interface KDModel<States extends Num> {
    Matrix<States, N1> getMin();

    Matrix<States, N1> getMax();

    /**
     * Distance, i.e. cost, between states. Note that this function is not, in
     * general, symmetric.
     * 
     * @return cost
     */
    double dist(Matrix<States, N1> start, Matrix<States, N1> end);

    void setStepNo(int stepNo);

    void setRadius(double radius);

    /**
     * return a point in the same direction as the input newConfig
     * relative to nearConfig, but only dist of the way there. Note this should
     * match the metric used by dist, which might be complicated for noneuclidean
     * spaces.
     * 
     * In general this function is intended to produce a feasible trajectory
     * starting at nearconfig; ideally ending at newConfig (double boundary problem)
     * but any trajectory will do.
     * 
     * @param stepNo     adjusts steering, choosing progressively closer nodes
     * @param nearConfig the nearest config
     * @param newConfig  the candidate config
     * @param dist       the fraction to go
     * @return steered new config
     */
    Matrix<States, N1> steer(KDNearNode<Node<States>> x_nearest, Matrix<States, N1> newConfig);

}
