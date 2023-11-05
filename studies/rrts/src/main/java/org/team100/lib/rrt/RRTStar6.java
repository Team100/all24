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
import org.team100.lib.random.MersenneTwister;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.space.SinglePath;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;

/**
 * RRT* version 6
 * 
 * This is the full-state field, two double integrators, so 4d altogether.
 * 
 * It uses the shooting solver, which is slow and unnecessary for this problem.
 */
public class RRTStar6<T extends KDModel<N4> & RobotModel<N4>> implements Solver<N4> {
    private static final boolean DEBUG = false;
    private static final double MAX_U = 2.5;
    private static final double DT = 0.6;
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.3;
    /** for testing */
    private static final boolean BIDIRECTIONAL = true;
    /** probability of branching */
    private static final double BUSHINESS = 0.2;

    private final T _model;
    private final double _gamma;
    private final Random random = new MersenneTwister(new Random().nextInt());
    private final ShootingSolver<N4, N2> solver = new ShootingSolver<>(VecBuilder.fill(MAX_U, MAX_U), DT, 20);
    private final Matrix<N4, N1> min;
    private final Matrix<N4, N1> max;

    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_b;

    // mutable loop variables to make the loop code cleaner
    private int stepNo;
    private double radius;
    private Path<N4> _sigma_best;

    public RRTStar6(T model, Sample<N4> sample, double gamma, KDNode<Node<N4>> T_a, KDNode<Node<N4>> T_b) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _T_a = T_a;
        _T_b = T_b;
        _gamma = gamma;
        min = _model.getMin();
        max = _model.getMax();
    }

    /** The top level is just a 2d double-integrator. */
    BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = (x, u) -> {
        double x2 = x.get(1, 0);
        double y2 = x.get(3, 0);
        double ux = u.get(0, 0);
        double uy = u.get(1, 0);
        double x1dot = x2;
        double x2dot = ux;
        double y1dot = y2;
        double y2dot = uy;
        Matrix<N4, N1> result = new Matrix<>(Nat.N4(), Nat.N1());
        result.set(0, 0, x1dot);
        result.set(1, 0, x2dot);
        result.set(2, 0, y1dot);
        result.set(3, 0, y2dot);
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

        LocalLink<N4> randLink = SampleFree(timeForward);
        if (DEBUG)
            System.out.println("randLink: " + randLink);

        if (CollisionFree(randLink.get_source().getState(), randLink.get_target().getState())) {
            // new node in "this" tree
            Node<N4> newNode = InsertNode(randLink, _T_a);
            if (DEBUG)
                System.out.println("NEW NODE " + newNode);
            List<NearNode<N4>> X_nearA = Near(newNode.getState(), _T_a);
            Rewire(X_nearA, newNode, timeForward);

            edges += 1;

            if (BIDIRECTIONAL) {
                // is there a point in the other tree that is reachable from
                // the node we just inserted? reachable nodes are nearby in a Euclidean
                // sense, though most Euclidean nearby nodes are not reachable.
                // so start with a list of near nodes and test them one by one.

                // near nodes in the other tree:
                List<NearNode<N4>> X_near = Near(newNode.getState(), _T_b);
                Matrix<N4, N1> x1 = newNode.getState();
                for (NearNode<N4> nearNode : X_near) {
                    // one near node in the other tree
                    Matrix<N4, N1> x2 = nearNode.node.getState();

                    ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2, timeForward);
                    if (sol != null) {
                        // there's a route from x1 aka newnode (in a) to x2 aka nearnode (in b)
                        if (DEBUG)
                            System.out.printf("FOUND feasible link x1: %s x2: %s sol: %s\n",
                                    Util.matStr(x1), Util.matStr(x2), sol);
                        // add a node in a corresponding to the near node in b
                        LocalLink<N4> newInA = new LocalLink<>(newNode, new Node<>(nearNode.node.getState()),
                                Math.abs(sol.dt));
                        Node<N4> newNewNode = InsertNode(newInA, _T_a);
                        Rewire(X_near, newNewNode, timeForward);
                        // create a path that traverses the new link.
                        Path<N4> p = GeneratePath(newNewNode, nearNode.node);
                        if (DEBUG)
                            System.out.println("PATH " + p);
                        if (_sigma_best == null) {
                            System.out.printf("first path distance %7.3f\n", p.getDistance());
                            _sigma_best = p;
                        } else {
                            if (p.getDistance() < _sigma_best.getDistance()) {
                                System.out.printf("new best path distance %7.3f\n", p.getDistance());
                                _sigma_best = p;
                            }
                        }
                        // don't need more than one feasible link
                        break;
                    }
                }
            }
        }
        if (BIDIRECTIONAL)
            SwapTrees();
        return edges;
    }

    public void SwapTrees() {
        KDNode<Node<N4>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     */
    Path<N4> GeneratePath(Node<N4> x_1, Node<N4> x_2) {
        if (!x_1.getState().isEqual(x_2.getState(), 0.001))
            throw new IllegalArgumentException(
                    "x1 " + x_1.getState().toString() + " != x2 " + x_2.getState().toString());

        Path<N4> p_1 = walkParents(new HashSet<>(), x_1);
        if (DEBUG)
            System.out.println("p1 " + p_1);
        Path<N4> p_2 = walkParents(new HashSet<>(), x_2);
        if (DEBUG)
            System.out.println("p2 " + p_2);
        List<Matrix<N4, N1>> states_2 = p_2.getStatesA();
        Collections.reverse(states_2);
        // don't include the same point twice
        states_2.remove(0);
        return new Path<>(p_1.getDistance() + p_2.getDistance(), p_1.getStatesA(), states_2);
    }

    boolean CollisionFree(Matrix<N4, N1> from, Matrix<N4, N1> to) {
        return _model.link(from, to);
    }

    /**
     * Applies random control to random tree node *in T_a.* If the node
     * has a parent, just continue the same way as that node.
     */
    LocalLink<N4> SampleFree(boolean timeForward) {
        // run backwards if the tree root is the goal
        if (DEBUG)
            System.out.println("SampleFree");
        while (true) {
            if (DEBUG)
                System.out.println("sample");
            // applied to a random point in the tree
            List<Node<N4>> nodes = KDTree.values(_T_a);
            int nodect = nodes.size();
            int nodeidx = random.nextInt(nodect);
            Node<N4> node_rand = nodes.get(nodeidx);
            // persuade the tree to be longer
            if (node_rand.getOutgoingCount() >= MAX_CHILDREN) {
                // maybe add anyway?
                if (random.nextDouble() > BUSHINESS) {
                    if (DEBUG)
                        System.out.println("skip node with too many children");
                    continue;
                }
            }

            double x_nearest1 = node_rand.getState().get(0, 0);
            double x_nearest2 = node_rand.getState().get(1, 0);
            double x_nearest3 = node_rand.getState().get(2, 0);
            double x_nearest4 = node_rand.getState().get(3, 0);

            Matrix<N4, N1> xxx = new Matrix<>(Nat.N4(), Nat.N1());
            xxx.set(0, 0, x_nearest1);
            xxx.set(1, 0, x_nearest2);
            xxx.set(2, 0, x_nearest3);
            xxx.set(3, 0, x_nearest4);

            Matrix<N2, N1> uuu = new Matrix<>(Nat.N2(), Nat.N1());
            double azimuth = 2 * Math.PI * random.nextDouble();
            uuu.set(0, 0, MAX_U * Math.cos(azimuth));
            uuu.set(1, 0, MAX_U * Math.sin(azimuth));

            double dt = DT * random.nextDouble();
            // maybe integrate backwards :-)
            if (!timeForward)
                dt *= -1.0;
            Matrix<N4, N1> newxxx = NumericalIntegration.rk4(f, xxx, uuu, dt);
            double x_new1 = newxxx.get(0, 0);
            double x_new2 = newxxx.get(1, 0);
            double x_new3 = newxxx.get(2, 0);
            double x_new4 = newxxx.get(3, 0);

            if (DEBUG)
                System.out.printf("integrated to get %s\n", Util.matStr(newxxx));

            // reject samples off the edge of the world
            if (x_new1 < min.get(0, 0) || x_new1 > max.get(0, 0)
                    || x_new2 < min.get(1, 0) || x_new2 > max.get(1, 0)
                    || x_new3 < min.get(2, 0) || x_new3 > max.get(2, 0)
                    || x_new4 < min.get(3, 0) || x_new4 > max.get(3, 0)

            ) {
                if (DEBUG)
                    System.out.printf("reject out of bounds %s u %s\n",
                            Util.matStr(newxxx), Util.matStr(uuu));
                continue;
            }

            Matrix<N4, N1> newConfig = newxxx;

            double[] dx_new = new double[] {
                    x_new1 - x_nearest1,
                    x_new2 - x_nearest2,
                    x_new3 - x_nearest3,
                    x_new4 - x_nearest4
            };
            if (node_rand.getIncoming() != null) {
                // continue in the same direction as the incoming,
                // to avoid clumping
                // could use the same "u" (or nearly same) here but
                // we want to allow the bang-bang deceleration case
                LinkInterface<N4> incoming = node_rand.getIncoming();
                double[] incoming_dx_new = new double[] {
                        incoming.get_target().getState().get(0, 0) - incoming.get_source().getState().get(0, 0),
                        incoming.get_target().getState().get(1, 0) - incoming.get_source().getState().get(1, 0),
                        incoming.get_target().getState().get(2, 0) - incoming.get_source().getState().get(2, 0),
                        incoming.get_target().getState().get(3, 0) - incoming.get_source().getState().get(3, 0)

                };
                double dot = incoming_dx_new[0] * dx_new[0]
                        + incoming_dx_new[1] * dx_new[1]
                        + incoming_dx_new[2] * dx_new[2]
                        + incoming_dx_new[3] * dx_new[3];
                if (dot < 0) {
                    if (DEBUG)
                        System.out.printf(
                                "reject dot parent [%5.3f %5.3f %5.3f %5.3f] node %s to %s u %s dot %5.3f\n",
                                incoming.get_source().getState().get(0, 0), incoming.get_source().getState().get(1, 0),
                                incoming.get_source().getState().get(2, 0), incoming.get_source().getState().get(3, 0),
                                Util.matStr(xxx),
                                Util.matStr(newxxx),
                                Util.matStr(uuu), dot);
                    continue;
                }

                // reject the new node if it's too close to any others
                // for now just use Euclidean distance.
                // note this will find the parent so make sure the step
                // size is larger than the buffer size
                KDNearNode<Node<N4>> n = KDTree.nearest(_model, _T_a, newConfig);
                if (n != null) {
                    // look only at spatial dimensions; it's ok for there to be lots of
                    // points at the same velocity.
                    double newDist = Math.sqrt(Math.pow(x_new1 - n._nearest.getState().get(0, 0), 2) +
                            Math.pow(x_new3 - n._nearest.getState().get(2, 0), 2));
                    if (newDist < BUFFER) {
                        if (DEBUG)
                            System.out.printf(
                                    "reject conflict from %s to %s old [%5.3f %5.3f %5.3f %5.3f] d %5.3f\n",
                                    Util.matStr(xxx),
                                    Util.matStr(newxxx),
                                    n._nearest.getState().get(0, 0), n._nearest.getState().get(1, 0),
                                    n._nearest.getState().get(2, 0), n._nearest.getState().get(3, 0),
                                    newDist);
                        continue;
                    }
                }

            }

            if (_model.clear(newConfig)) {
                if (DEBUG)
                    System.out.printf(
                            "found new %s u %s dt %5.3f\n",
                            Util.matStr(newxxx), Util.matStr(uuu), dt);
                // note abs() due to (sometimes) time reversal
                return new LocalLink<>(node_rand, new Node<>(newConfig), Math.abs(dt));
            }
            if (DEBUG)
                System.out.println("not clear");
        }
    }

    /**
     * Return the nearest node in the tree, using the KDTree metric, which
     * could be very wrong but is probably useful.
     * 
     * @param x_rand   the random sample
     * @param rootNode the tree to look through
     */
    KDNearNode<Node<N4>> Nearest(Matrix<N4, N1> x_rand, KDNode<Node<N4>> rootNode) {
        // for now just do it the dumb way
        List<Node<N4>> nodes = KDTree.values(rootNode);
        Node<N4> bestNode = null;
        double bestDistance = Double.MAX_VALUE;
        for (Node<N4> node : nodes) {
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
    Matrix<N4, N1> Steer(KDNearNode<Node<N4>> x_nearest, Matrix<N4, N1> x_rand) {
        _model.setStepNo(stepNo);
        // _model.setRadius(radius);
        _model.setRadius(same(_T_a.getValue().getState(), _model.initial()) ? 1 : -1);
        return _model.steer(x_nearest, x_rand);
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. Returns the
     * single
     * nearest node if there are no other near nodes.
     */
    ArrayList<NearNode<N4>> Near(Matrix<N4, N1> x_new, KDNode<Node<N4>> rootNode) {
        ArrayList<NearNode<N4>> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> nearNodes.add(new NearNode<>(node, dist)));
        return nearNodes;
    }

    /**
     * Returns a member of X_near resulting in the lowest-cost path to x_new.
     * Removes infeasible nodes from X_near so we don't look at them again later.
     */
    Node<N4> ChooseParent(List<NearNode<N4>> X_near, Matrix<N4, N1> x_new) {
        Collections.sort(X_near);
        Iterator<NearNode<N4>> ni = X_near.iterator();
        while (ni.hasNext()) {
            NearNode<N4> nearNode = ni.next();
            ni.remove();
            if (CollisionFree(nearNode.node.getState(), x_new)) {
                return nearNode.node;
            }
        }
        return null;
    }

    /** Add the node link.target to the tree, with an edge from source to target. */
    Node<N4> InsertNode(LocalLink<N4> link, KDNode<Node<N4>> rootNode) {
        Graph.newLink(link.get_source(), link.get_target(), link.get_linkDist());
        KDTree.insert(_model, rootNode, link.get_target());
        return link.get_target();
    }

    /**
     * look through the nodes in X_near to see if any should be new children of
     * newNode.
     */
    void Rewire(List<NearNode<N4>> X_near, Node<N4> newNode, boolean timeForward) {
        if (DEBUG)
            System.out.printf("Rewire candidates %d\n", X_near.size());
        ListIterator<NearNode<N4>> li = X_near.listIterator(X_near.size());
        while (li.hasPrevious()) {
            NearNode<N4> jn = li.previous();
            if (jn.node.getIncoming() != null) {
                Matrix<N4, N1> x1 = newNode.getState();
                Matrix<N4, N1> x2 = jn.node.getState();
                // shortcut the inevitable duplicate
                if (x1.isEqual(x2, 0.01))
                    continue;
                if (DEBUG)
                    System.out.printf("Try rewiring %s to %s\n", Util.matStr(x1), Util.matStr(x2));
                ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1, x2, timeForward);
                if (sol == null) {
                    if (DEBUG)
                        System.out.println("no solution");
                } else {
                    if (Graph.rewire(_model, newNode, jn.node, Math.abs(sol.dt))) {
                        if (DEBUG)
                            System.out.println("REWIRED");
                    }
                }
            }
        }
    }

    @Override
    public List<Node<N4>> getNodesA() {
        ArrayList<Node<N4>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_a));
        return allNodes;
    }

    @Override
    public List<Node<N4>> getNodesB() {
        ArrayList<Node<N4>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_b));
        return allNodes;
    }

    @Override
    public Path<N4> getBestPath() {
        return _sigma_best;
    }

    /**
     * Starting from leaf node, walk the parent links to accumulate
     * the full path, and reverse it, to return a path from root to node.
     * 
     * populates list A only. bleah.
     * 
     * @param node leaf node
     */
    Path<N4> walkParents(Set<Node<N4>> visited, Node<N4> node) {
        // Collect the states along the path (backwards)
        List<Matrix<N4, N1>> configs = new LinkedList<>();
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
            LinkInterface<N4> incoming = node.getIncoming();
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
        this.radius = _gamma * Math.pow(Math.log(stepNo + 1.0) / (stepNo + 1), 0.25);
    }

    static boolean same(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        return a.isEqual(b, 0.0001);
    }

    @Override
    public SinglePath<N4> getBestSinglePath() {
        throw new UnsupportedOperationException("Unimplemented method 'getBestSinglePath'");
    }
}