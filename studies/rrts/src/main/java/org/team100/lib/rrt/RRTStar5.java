package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.Set;
import java.util.function.BiFunction;

import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.LocalLink;
import org.team100.lib.graph.NearNode;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDModel;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.math.ShootingSolver;
import org.team100.lib.planner.RobotModel;
import org.team100.lib.planner.Solver;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.space.SinglePath;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * RRT* version 5.
 * 
 * For the non-Euclidean spaces, state sampling doesn't seem to work
 * very well; there's no good way to find "nearby" states, and the
 * admissible fraction is tiny, especially with a weak control.
 * 
 * So instead, sample control space and apply to a random node.
 * 
 * maybe try
 */
public class RRTStar5<T extends KDModel<N2> & RobotModel<N2>> implements Solver<N2> {
    private static final boolean DEBUG = false;
    private static final double MAX_U = 2.5;
    private static final double DT = 0.1;
    private static final double l = 1; // length meter
    private static final double _g = 9.81; // gravity m/s/s
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.025;
    /** probability of branching */
    private static final double BUSHINESS = 0.001;
    private static final boolean bidirectional = true;

    private final T _model;
    private final double _gamma;
    private final Random random = new Random();
    private final Matrix<N2, N1> min;
    private final Matrix<N2, N1> max;
    private final ShootingSolver<N2, N1> solver = new ShootingSolver<>(VecBuilder.fill(MAX_U), DT, 20);

    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node<N2>> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node<N2>> _T_b;

    // mutable loop variables to make the loop code cleaner
    private int stepNo;
    private double radius;
    private Path<N2> _sigma_best;

    public RRTStar5(T model, Sample<N2> sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _T_a = new KDNode<>(new Node<>(model.initial()));
        _T_b = new KDNode<>(new Node<>(model.goal()));
        _gamma = gamma;
        min = _model.getMin();
        max = _model.getMax();
    }

    // accurate integration is absolutely required here,
    // otherwise the dynamics are wrong
    BiFunction<Matrix<N2, N1>, Matrix<N1, N1>, Matrix<N2, N1>> f = (xx, uu) -> {
        double xx1 = xx.get(0, 0);
        double xx2 = xx.get(1, 0);
        double xx1dot = xx2;
        double xx2dot = -1 * _g * Math.sin(xx1) / l + uu.get(0, 0);
        Matrix<N2, N1> result = new Matrix<>(Nat.N2(), Nat.N1());
        result.set(0, 0, xx1dot);
        result.set(1, 0, xx2dot);
        return result;
    };

    /**
     * Note this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it doesn't use Extend, so it doesn't try to connect unless
     * a new node is actually inserted.
     * 
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        if (DEBUG)
            System.out.println("step");
        int edges = 0;

        // the sample is now control-sampled, guaranteed feasible.
        boolean timeForward = same(_T_a.getValue().getState(), _model.initial());

        LocalLink<N2> randLink = SampleFree(timeForward);
        if (DEBUG)
            System.out.println("randLink: " + randLink);

        if (CollisionFree(randLink.get_source().getState(), randLink.get_target().getState())) {
            // X_near uses Euclidean metric, so do the distance over
            // actually for now don't use this near-search thing at all

            // new node in "this" tree
            Node<N2> newNode = InsertNode(randLink, _T_a);
            if (DEBUG)
                System.out.println(newNode);
            List<NearNode<N2>> X_nearA = Near(newNode.getState(), _T_a);
            Rewire(X_nearA, newNode, timeForward);

            edges += 1;

            if (bidirectional) {
                // is there a point in the other tree that is reachable from
                // the node we just inserted? reachable nodes are nearby in a Euclidean
                // sense, though most Euclidean nearby nodes are not reachable.
                // so start with a list of near nodes and test them one by one.

                // near nodes in the other tree:
                List<NearNode<N2>> X_near = Near(newNode.getState(), _T_b);
                Matrix<N2, N1> x1 = VecBuilder.fill(newNode.getState().get(0, 0), newNode.getState().get(1, 0));
                for (NearNode<N2> nearNode : X_near) {
                    // one near node in the other tree
                    Matrix<N2, N1> x2 = VecBuilder.fill(nearNode.node.getState().get(0, 0),
                            nearNode.node.getState().get(1, 0));
                    ShootingSolver<N2, N1>.Solution sol = solver.solve(Nat.N2(), Nat.N1(), f, x1, x2, timeForward);
                    if (sol != null) {
                        // there's a route from x1 aka newnode (in a) to x2 aka nearnode (in b)
                        if (DEBUG)
                            System.out.printf("FOUND feasible link x1: %s x2: %s sol: %s\n",
                                    Util.matStr(x1), Util.matStr(x2), sol);
                        // add a node in a corresponding to the near node in b
                        LocalLink<N2> newInA = new LocalLink<>(newNode, new Node<>(nearNode.node.getState()),
                                Math.abs(sol.dt));
                        Node<N2> newNewNode = InsertNode(newInA, _T_a);
                        // Rewire(X_near, newNode);
                        // return GeneratePath(x_1, newNode);
                        // create a path that traverses the new link.
                        Path<N2> p = GeneratePath(newNewNode, nearNode.node);
                        if (DEBUG)
                            System.out.println("PATH " + p);
                        if (_sigma_best == null) {
                            _sigma_best = p;
                        } else {
                            if (p.getDistance() < _sigma_best.getDistance()) {
                                _sigma_best = p;
                            }
                        }
                        // don't need more than one feasible link
                        break;
                    }
                }
            }
        }
        if (bidirectional)
            SwapTrees();
        return edges;
    }

    public void SwapTrees() {
        KDNode<Node<N2>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     */
    Path<N2> GeneratePath(Node<N2> x_1, Node<N2> x_2) {
        if (!x_1.getState().isEqual(x_2.getState(), 0.001))
            throw new IllegalArgumentException(
                    "x1 " + x_1.getState().toString() + " != x2 " + x_2.getState().toString());
        Path<N2> p_1 = walkParents(new HashSet<>(), x_1);
        Path<N2> p_2 = walkParents(new HashSet<>(), x_2);
        List<Matrix<N2, N1>> states_2 = p_2.getStatesA();
        Collections.reverse(states_2);
        // don't include the same point twice
        states_2.remove(0);
        return new Path<>(p_1.getDistance() + p_2.getDistance(), p_1.getStatesA(), states_2);
    }

    boolean CollisionFree(Matrix<N2, N1> from, Matrix<N2, N1> to) {
        return _model.link(from, to);
    }

    /**
     * Applies random control to random tree node *in T_a.* If the node
     * has a parent, just continue the same way as that node.
     */
    LocalLink<N2> SampleFree(boolean timeForward) {
        // run backwards if the tree root is the goal

        while (true) {
            if (DEBUG)
                System.out.println("sample");
            // applied to a random point in the tree
            List<Node<N2>> nodes = KDTree.values(_T_a);
            int nodect = nodes.size();
            int nodeidx = random.nextInt(nodect);
            Node<N2> node_rand = nodes.get(nodeidx);
            // persuade the tree to be longer
            if (node_rand.getOutgoingCount() >= MAX_CHILDREN) {
                // maybe add anyway?
                if (random.nextDouble() > BUSHINESS)
                    continue;
            }

            double x_nearest1 = node_rand.getState().get(0, 0);
            double x_nearest2 = node_rand.getState().get(1, 0);

            // double x1dot = x_nearest2;
            // random control
            // double u = MIN_U + (MAX_U - MIN_U) * random.nextDouble();
            double u = MAX_U + random.nextGaussian(0, MAX_U / 10);
            if (u > MAX_U)
                u = -2.0 * MAX_U + u;
            u = Math.max(-1.0 * MAX_U, u);
            u = Math.min(MAX_U, u);
            // System.out.println("U " + u);
            // double u = 0;
            // double x2dot = -1 * _g * Math.sin(x_nearest1) / l + u;
            double dt = DT;
            if (!timeForward)
                dt *= -1.0;

            double x_new1;
            double x_new2;

            Matrix<N2, N1> xxx = new Matrix<>(Nat.N2(), Nat.N1());
            xxx.set(0, 0, x_nearest1);
            xxx.set(1, 0, x_nearest2);
            Matrix<N1, N1> uuu = new Matrix<>(Nat.N1(), Nat.N1());
            uuu.set(0, 0, u);
            Matrix<N2, N1> newxxx = NumericalIntegration.rk4(f, xxx, uuu, dt);

            x_new1 = newxxx.get(0, 0);
            x_new2 = newxxx.get(1, 0);

            // reject samples off the edge of the world
            if (x_new1 < min.get(0, 0) || x_new1 > max.get(0, 0) || x_new2 < min.get(1, 0) || x_new2 > max.get(1, 0)) {
                if (DEBUG)
                    System.out.printf(
                            "reject out of bounds [%5.3f %5.3f] u %5.3f\n",
                            x_new1, x_new2, u);
                continue;
            }

            Matrix<N2, N1> newConfig = newxxx;

            double[] dx_new = new double[] { x_new1 - x_nearest1, x_new2 - x_nearest2 };
            if (node_rand.getIncoming() != null) {
                // continue in the same direction as the incoming,
                // to avoid clumping
                // could use the same "u" (or nearly same) here but
                // we want to allow the bang-bang deceleration case
                LinkInterface<N2> incoming = node_rand.getIncoming();
                double[] incoming_dx_new = new double[] {
                        incoming.get_target().getState().get(0, 0) - incoming.get_source().getState().get(0, 0),
                        incoming.get_target().getState().get(1, 0) - incoming.get_source().getState().get(1, 0)
                };
                double dot = incoming_dx_new[0] * dx_new[0]
                        + incoming_dx_new[1] * dx_new[1];
                if (dot < 0) {
                    if (DEBUG)
                        System.out.printf(
                                "reject dot parent [%5.3f %5.3f] node [%5.3f %5.3f] to [%5.3f %5.3f] u %5.3f dot %5.3f\n",
                                incoming.get_source().getState().get(0, 0), incoming.get_source().getState().get(1, 0),
                                x_nearest1, x_nearest2,
                                x_new1, x_new2,
                                u, dot);
                    continue;
                }

                // reject the new node if it's too close to any others
                // for now just use Euclidean distance.
                // note this will find the parent so make sure the step
                // size is larger than the buffer size
                KDNearNode<Node<N2>> n = KDTree.nearest(_model, _T_a, newConfig);
                if (n != null) {

                    double newDist = Math.sqrt(Math.pow(x_new1 - n._nearest.getState().get(0, 0), 2) +
                            Math.pow(x_new2 - n._nearest.getState().get(1, 0), 2));
                    if (newDist < BUFFER) {
                        if (DEBUG)
                            System.out.printf(
                                    "reject conflict from [%5.3f %5.3f] to [%5.3f %5.3f] old [%5.3f %5.3f] d %5.3f\n",
                                    x_nearest1, x_nearest2,
                                    x_new1, x_new2,
                                    n._nearest.getState().get(0, 0), n._nearest.getState().get(1, 0),
                                    newDist);
                        continue;
                    }
                }

            }

            if (_model.clear(newConfig)) {
                if (DEBUG)
                    System.out.printf(
                            "found new [%5.3f %5.3f] u %5.3f dt %5.3f\n",
                            x_new1, x_new2, u, dt);
                return new LocalLink<>(node_rand, new Node<>(newConfig), DT);
            }
        }
    }

    /**
     * Return the nearest node in the tree, using the KDTree metric, which
     * could be very wrong but is probably useful.
     * 
     * @param x_rand   the random sample
     * @param rootNode the tree to look through
     */
    KDNearNode<Node<N2>> Nearest(Matrix<N2, N1> x_rand, KDNode<Node<N2>> rootNode) {
        // for now just do it the dumb way
        List<Node<N2>> nodes = KDTree.values(rootNode);
        Node<N2> bestNode = null;
        double bestDistance = Double.MAX_VALUE;
        for (Node<N2> node : nodes) {
            double d = _model.dist(node.getState(), x_rand);
            if (d <= 0)
                continue;
            if (d < bestDistance) {
                bestDistance = d;
                bestNode = node;
            }
        }
        return new KDNearNode<>(bestDistance, bestNode);
    }

    /**
     * Return a state that tries to go from x_nearest to x_rand using a feasible
     * trajectory.
     * 
     * @param x_nearest starting state
     * @param x_rand    goal state
     * @return x_new a feasible state
     */
    Matrix<N2, N1> Steer(KDNearNode<Node<N2>> x_nearest, Matrix<N2, N1> x_rand) {
        _model.setStepNo(stepNo);
        _model.setRadius(same(_T_a.getValue().getState(), _model.initial()) ? 1 : -1);
        return _model.steer(x_nearest, x_rand);
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. Returns the
     * single
     * nearest node if there are no other near nodes.
     */
    List<NearNode<N2>> Near(Matrix<N2, N1> x_new, KDNode<Node<N2>> rootNode) {
        List<NearNode<N2>> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> nearNodes.add(new NearNode<>(node, dist)));
        return nearNodes;
    }

    /**
     * Returns a member of X_near resulting in the lowest-cost path to x_new.
     * Removes infeasible nodes from X_near so we don't look at them again later.
     */
    Node<N2> ChooseParent(List<NearNode<N2>> X_near, Matrix<N2, N1> x_new) {
        Collections.sort(X_near);
        Iterator<NearNode<N2>> ni = X_near.iterator();
        while (ni.hasNext()) {
            NearNode<N2> nearNode = ni.next();
            ni.remove();
            if (CollisionFree(nearNode.node.getState(), x_new)) {
                return nearNode.node;
            }
        }
        return null;
    }

    /** Add the node link.target to the tree, with an edge from source to target. */
    Node<N2> InsertNode(LocalLink<N2> link, KDNode<Node<N2>> rootNode) {
        Graph.newLink(link.get_source(), link.get_target(), link.get_linkDist());
        KDTree.insert(_model, rootNode, link.get_target());
        return link.get_target();
    }

    /**
     * look through the nodes in X_near to see if any should be new children of
     * newNode.
     */
    void Rewire(List<NearNode<N2>> X_near, Node<N2> newNode, boolean timeForward) {
        ListIterator<NearNode<N2>> li = X_near.listIterator(X_near.size());
        while (li.hasPrevious()) {
            NearNode<N2> jn = li.previous();
            if (jn.node.getIncoming() != null) {
                Matrix<N2, N1> x1 = newNode.getState();
                Matrix<N2, N1> x2 = jn.node.getState();
                ShootingSolver<N2, N1>.Solution sol = solver.solve(Nat.N2(), Nat.N1(), f, x1, x2, timeForward);
                if (sol != null) {
                    if (Graph.rewire(_model, newNode, jn.node, Math.abs(sol.dt))) {
                        if (DEBUG)
                            System.out.println("REWIRED");
                    }

                }

            }
        }
    }

    @Override
    public List<Node<N2>> getNodesA() {
        List<Node<N2>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.<N2, Node<N2>>values(_T_a));
        return allNodes;
    }

    @Override
    public List<Node<N2>> getNodesB() {
        List<Node<N2>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_b));
        return allNodes;
    }

    @Override
    public Path<N2> getBestPath() {
        return _sigma_best;
    }

    /**
     * Starting from leaf node, walk the parent links to accumulate
     * the full path, and reverse it, to return a path from root to node.
     * 
     * @param node leaf node
     */
    Path<N2> walkParents(Set<Node<N2>> visited, Node<N2> node) {
        // Collect the states along the path (backwards)
        List<Matrix<N2, N1>> configs = new LinkedList<>();
        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance
        double totalDistance = 0;
        while (true) {
            if (visited.contains(node)) {
                System.out.println("found a cycle");
                throw new IllegalArgumentException();
            }
            visited.add(node);
            configs.add(node.getState());
            LinkInterface<N2> incoming = node.getIncoming();
            if (incoming == null)
                break;
            totalDistance += incoming.get_linkDist();
            node = incoming.get_source();
        }
        // now we have the forwards list of states
        Collections.reverse(configs);

        return new Path<>(totalDistance, configs, new LinkedList<>());
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
        this.radius = _gamma * Math.pow(Math.log(stepNo + 1.0) / (stepNo + 1), 0.5);
    }

    static boolean same(Matrix<N2, N1> a, Matrix<N2, N1> b) {
        return a.isEqual(b, 0.0001);
    }

    @Override
    public SinglePath<N2> getBestSinglePath() {
        throw new UnsupportedOperationException("Unimplemented method 'getBestSinglePath'");
    }
}