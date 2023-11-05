package org.team100.lib.rrt.example.swingup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * 
 * This version does not use LQR math.
 * 
 * Single jointed pendulum
 * 
 * Example of non-euclidean space.
 * 
 * x1 = angle from downward
 * x2 = velocity
 * 
 * x1dot = x2
 * x2dot = - g * sin(x1) / l + u
 * 
 * let's say g = l.
 * 
 * A = [ 0 1, -sin x1 0]
 * B = [0, 1]
 * 
     * zeroth dimension is position (radians, down is zero).
     * first dimension is velocity (radians per second).
      */
public class PendulumArena2 implements Arena<N2> {
    
    private static final double MIN_U = -0.5;
    private static final double MAX_U = 0.5;
    private static final double MIN_DT = 0.01;
    private static final double MAX_DT = 0.25;

    private static final double POSITION_TOLERANCE = 0.25;
    private static final double VELOCITY_TOLERANCE = 0.25;

    private final Matrix<N2, N1> _init;
    private final Matrix<N2, N1> _goal;
    /** same as the paper */
    private static final Matrix<N2, N1> _min = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { -4, -8 });
    private static final Matrix<N2, N1> _max = new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 4, 8 });

    Obstacle[] _obstacles = new Obstacle[] {};

    double h = 0.1; // TODO: this is surely wrong. what time interval to use?

    // see pend_rrt.m
    private static final double l = 1; // length meter
    private final double _g; // gravity m/s/s

    // private int stepNo;
    // private double radius;
    private boolean timeForward = true;

    public PendulumArena2(Matrix<N2, N1> init, Matrix<N2, N1> goal, double gravity) {
        _init = init;
        _goal = goal;
        _g = gravity;

    }

    @Override
    public Matrix<N2, N1> getMin() {
        return _min.copy();
    }

    @Override
    public Matrix<N2, N1> getMax() {
        return _max.copy();
    }

    /**
     * try to make this return time.
     * 
     * x1dot = x2
     * x2dot = something + u
     */
    @Override
    public double dist(Matrix<N2, N1> start, Matrix<N2, N1> end) {
        double x_start1 = start.get(0, 0);
        double x_start2 = start.get(1, 0);
        double x_end1 = end.get(0, 0);
        double x_end2 = end.get(1, 0);
        // TODO: what should x1dot actually be?
        double x1dot = (x_start2 + x_end2) / 2;
        double dx1 = x_end1 - x_start1;
        return dx1 / x1dot;
    }

    @Override
    public void setStepNo(int stepNo) {
        // this.stepNo = stepNo;
    }

    // for now this means the direction of time
    // TODO: a different way to do that
    @Override
    public void setRadius(double radius) {
        timeForward = (radius > 0);
    }

    /**
     * steer from the near config towards the new config using the real dynamics and
     * bang-bang control, i.e. find the u value to try to go from the current
     * point to the new point, and then integrate forward to find the new point.
     * 
     * Returns null if there is no easy way to get to the goal state.
     * 
     * @param x_nearest starting state
     * @param x_rand    goal state
     * @return x_new a feasible state
     */
    @Override
    public Matrix<N2, N1> steer(KDNearNode<Node<N2>> x_nearest, Matrix<N2, N1> x_rand) {

        if (x_nearest._nearest == null) {
            return null;
        }
        Matrix<N2, N1> x_nearest_state = x_nearest._nearest.getState();
        double x_nearest1 = x_nearest_state.get(0, 0);
        double x_nearest2 = x_nearest_state.get(1, 0);

        double x_rand1 = x_rand.get(0, 0);
        double x_rand2 = x_rand.get(1, 0);

        // from system dynamics
        double x1dot = x_nearest2;

        double dx1 = x_rand1 - x_nearest1;
        // xdot = dx/dt so dt = dx/xdot
        double dt = dx1 / x1dot;
        if (timeForward) {
            if (dt <= MIN_DT)
                return null;
            if (dt > MAX_DT)
                dt = MAX_DT;
        } else {
            if (dt >= -1.0 * MIN_DT)
                return null;
            if (dt < -1.0 * MAX_DT)
                dt = -1.0 * MAX_DT;
        }

        // from system dynamics
        // x2dot = - g * sin(x1) / l + u
        // u = dx2/dt + g*sin(x1)

        double dx2 = x_rand2 - x_nearest2;
        double u = dx2 / dt + _g * Math.sin(x_nearest1) / l;
        u = Math.max(MIN_U, u);
        u = Math.min(MAX_U, u);

        double x2dot = -1 * _g * Math.sin(x_nearest1) / l + u;

        double x_new1 = x_nearest1 + x1dot * dt + 0.5 * x2dot * dt * dt;
        double x_new2 = x_nearest2 + x2dot * dt;

        System.out.printf("from [%5.3f %5.3f] to [%5.3f %5.3f] xdot [%5.3f %5.3f] dt %5.3f u %5.3f\n",
                x_nearest1, x_nearest2, x_new1, x_new2, x1dot, x2dot, dt, u);
        return new Matrix<>(Nat.N2(), Nat.N1(), new double[] { x_new1, x_new2 });
    }

    @Override
    public Matrix<N2, N1> initial() {
        return _init;
    }

    @Override
    public Matrix<N2, N1> goal() {
        return _goal;
    }

    @Override
    public boolean goal(Matrix<N2, N1> config) {
        if (Math.abs(config.get(0, 0) - _goal.get(0, 0)) > POSITION_TOLERANCE)
            return false;
        if (Math.abs(config.get(1, 0) - _goal.get(1, 0)) > VELOCITY_TOLERANCE)
            return false;
        return true;
    }

    /** There aren't really obstacles, but this will come in handy later. */
    @Override
    public boolean clear(Matrix<N2, N1> config) {
        return true;
    }

    /**
     * This checks for obstacles along the path. between a and b but since there
     * aren't any obstacles.
     */
    @Override
    public boolean link(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return true;
    }

    public List<Obstacle> obstacles() {
        return Arrays.asList(_obstacles);
    }

}
