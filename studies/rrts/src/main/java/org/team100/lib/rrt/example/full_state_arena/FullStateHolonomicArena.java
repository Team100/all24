package org.team100.lib.rrt.example.full_state_arena;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.team100.lib.example.Arena;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.geom.Polygon;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/**
 * this only works for 4d full state
 */
public class FullStateHolonomicArena implements Arena<N4> {
    private static final double DISCRETIZATION = 0.25;
    private static final double ROBOT_RADIUS = .4;
    private static final double GOAL_RADIUS = 0.4;

    // realistic initial and goal states
    private static final Matrix<N4, N1> _init = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 15.5, 0, 6.75, 0 });
    private static final Matrix<N4, N1> _goal = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1.93, 0, 2.748, 0 });

    // closer initial/goal for testing
    // private static final Matrix<N4, N1> _init = new Matrix<>(Nat.N4(), Nat.N1(),
    // new double[] { 10, 0, 4, 0 });
    // private static final Matrix<N4, N1> _goal = new Matrix<>(Nat.N4(), Nat.N1(),
    // new double[] { 6, 0, 2, 0 });

    // conservative speed limit
    // private static final Matrix<N4, N1> _min = new Matrix<>(Nat.N4(), Nat.N1(),
    // new double[] { 0, -3, 0, -3 });
    // private static final Matrix<N4, N1> _max = new Matrix<>(Nat.N4(), Nat.N1(),
    // new double[] { 16, 3, 8, 3 });

    // higher speed limit
    private static final Matrix<N4, N1> _min = new Matrix<>(Nat.N4(), Nat.N1(),
            new double[] { 0, -4, 0, -4 });
    private static final Matrix<N4, N1> _max = new Matrix<>(Nat.N4(), Nat.N1(),
            new double[] { 16, 4, 8, 4 });

    // used for steering
    // private final double _gamma;
    // private int stepNo;
    private double radius;

    final List<Obstacle> _obstacles = new ArrayList<>();

    double dia = 0.5;

    Random rand = new Random();

    public FullStateHolonomicArena() {
        this(0);
    }

    public FullStateHolonomicArena(int i) {
        double t = 0.1*i;

        // fixed obstacles
        // nodes
        _obstacles.add(new Polygon(Color.RED, 0, 0, 1.43, 0, 1.43, 5.49, 0, 5.49));
        // community
        _obstacles.add(new Polygon(Color.BLUE, 13.18, 0, 16, 0, 16, 5.49, 13.18, 5.49));
        // opponents

        // loading
        _obstacles.add(new Polygon(Color.BLUE, 0, 8, 3.36, 8, 3.36, 5.49, 0, 5.49));
        // charge stations
        _obstacles.add(new Polygon(Color.RED, 2.98, 1.51, 4.91, 1.51, 4.91, 3.98, 2.98, 3.98));
        _obstacles.add(new Polygon(Color.BLUE, 11.63, 1.51, 13.56, 1.51, 13.56, 3.98, 11.63, 3.98));

        // moving obstacles
        addBot(Color.BLUE, -4*Math.sin(t)+7.5, 3*Math.sin(t)+4);
        addBot(Color.BLUE, -2*Math.sin(2*t+1)+6.5, Math.sin(2*t+1)+6);
        addBot(Color.BLUE, 3*Math.sin(2*t)+9.5, Math.sin(0.5*t)+6.5);
        // alliance-mate
        addBot(Color.RED, 3*Math.sin(t+2)+6, Math.sin(Math.sqrt(3)*t+2)+5);
        addBot(Color.RED, 4*Math.sin(3*t)+9, Math.sin(3*t)+6.5);
    }

    void addBot(Color color, double x, double y) {
      //  x += 2 * rand.nextDouble() - 1.0;
        //y += 2 * rand.nextDouble() - 1.0;
        _obstacles.add(new Polygon(color, x - dia, y - dia, x + dia, y - dia, x + dia, y + dia, x - dia, y + dia));

    }

    @Override
    public Matrix<N4, N1> getMin() {
        return _min.copy();
    }

    @Override
    public Matrix<N4, N1> getMax() {
        return _max.copy();
    }

    @Override
    public double dist(Matrix<N4, N1> start, Matrix<N4, N1> end) {
        return start.minus(end).normF();
    }

    @Override
    public void setStepNo(int stepNo) {
        // this.stepNo = stepNo;
    }

    @Override
    public void setRadius(double radius) {
        this.radius = radius;
    }

    @Override
    public Matrix<N4, N1> steer(KDNearNode<Node<N4>> x_nearest, Matrix<N4, N1> x_rand) {
        double dist = radius / x_nearest._dist;
        // if it's close enough then just return it
        if (x_nearest._dist < radius)
            return x_rand;
        Matrix<N4, N1> nearConfig = x_nearest._nearest.getState();
        return nearConfig.plus(x_rand.minus(nearConfig).times(dist));
    }

    /**
     * config is (x xdot y ydot)
     */
    @Override
    public boolean clear(Matrix<N4, N1> config) {
        if (config.get(0, 0) - ROBOT_RADIUS < _min.get(0, 0))
            return false;
        if (config.get(2, 0) - ROBOT_RADIUS < _min.get(2, 0))
            return false;
        if (config.get(0, 0) + ROBOT_RADIUS > _max.get(0, 0))
            return false;
        if (config.get(2, 0) + ROBOT_RADIUS > _max.get(2, 0))
            return false;
        // poor-man's velocity limit. TODO: add bang-cruise-bang solutions.
        if (config.get(1, 0) < _min.get(1, 0)) {
            // System.out.printf("%f %f\n", config.get(1, 0), _min.get(1, 0));
            return false;
        }
        if (config.get(3, 0) < _min.get(3, 0)) {
            // System.out.printf("%f %f\n", config.get(3, 0), _min.get(1, 0));
            return false;
        }
        if (config.get(1, 0) > _max.get(1, 0)) {
            // System.out.printf("%f %f\n", config.get(1, 0), _max.get(1, 0));
            return false;
        }
        if (config.get(3, 0) > _max.get(3, 0)) {
            // System.out.printf("%f %f\n", config.get(3, 0), _max.get(1, 0));
            return false;
        }

        // robot-obstacle collision
        for (Obstacle obstacle : _obstacles) {
            if (obstacle.distToPoint(config.get(0, 0), config.get(2, 0)) < ROBOT_RADIUS)
                return false;
        }
        return true;
    }

    @Override
    public boolean link(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        double dist = b.minus(a).normF();
        int steps = (int) Math.floor(dist / DISCRETIZATION) + 2;
        for (int i = 0; i <= steps; ++i) {
            Matrix<N4, N1> p = a.times(steps - i).plus(b.times(i)).div(steps);
            if (!clear(p)) {
                return false;
            }
        }
        return true;
    }

    @Override
    public Matrix<N4, N1> initial() {
        return _init;
    }

    @Override
    public Matrix<N4, N1> goal() {
        return _goal;
    }

    @Override
    public boolean goal(Matrix<N4, N1> conf) {
        return dist(conf, _goal) < GOAL_RADIUS;
    }

    public List<Obstacle> obstacles() {
        return _obstacles;
    }
}
