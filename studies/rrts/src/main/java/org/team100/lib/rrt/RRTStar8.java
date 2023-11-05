package org.team100.lib.rrt;

import static java.lang.Math.sqrt;

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
import java.util.function.Predicate;

import org.team100.lib.example.Arena;
import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.LocalLink;
import org.team100.lib.graph.NearNode;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.math.ShootingSolver;
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
 * RRT* version 8
 * 
 * This is like version 7, but with the PLP interpolator, not yet finished.
 * 
 * Full-state 4d field, following the bang-bang rrt paper.
 * 
 * Papers referenced below:
 * 
 * [1] LaSalle et al, Bang-Bang RRT, 2023. https://arxiv.org/pdf/2210.01744.pdf
 * [2] Hauser et al, Optimal shortcuts, 2010,
 * https://motion.cs.illinois.edu/papers/icra10-smoothing.pdf
 * 
 * alpha = sample from state
 * x_n = nearest neighbor
 * find a path to the steer to sample using max U
 * make the times of each axis match, respecting the gap if it exists
 * same routine for rewiring i guess?
 * 
 * Note B Paden wrote a paper in 2017 about nearest-neighbor finding for
 * nonholonomic systems, maybe useful.
 * 
 * 
 * 
 * for rewiring and connecting, use a linear solver.
 */
public class RRTStar8<T extends Arena<N4>> implements Solver<N4> {
    public static boolean DEBUG = false;
    private static final double MAX_U = 2.5;
    private static final double DT = 0.6;
    private static final int MAX_CHILDREN = 1;
    private static final double BUFFER = 0.3;
    /** for testing */
    private static final boolean BIDIRECTIONAL = true;
    /** probability of branching */
    private static final double BUSHINESS = 0.2;

    private final T _model;
    private final Sample<N4> _sample;
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
    // TODO remove
    private Path<N4> _sigma_best;
    private SinglePath<N4> _single_sigma_best;

    public RRTStar8(T model, Sample<N4> sample, double gamma, KDNode<Node<N4>> T_a, KDNode<Node<N4>> T_b) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _sample = sample;
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

    public boolean curves = true;

    /**
     * Note this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it doesn't use Extend, so it doesn't try to connect unless
     * a new node is actually inserted.
     * 
     * @param curves add lots of little segments
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        if (DEBUG)
            System.out.println("step");
        int edges = 0;

        boolean timeForward = same(_T_a.getValue().getState(), _model.initial());

        // alpha
        Matrix<N4, N1> x_rand = SampleState();

        // x_n
        KDNearNode<Node<N4>> x_nearestA = BangBangNearest(x_rand, _T_a, timeForward);
        if (x_nearestA == null)
            return 0;

        // includes states and controls
        Trajectory phiA = BangBangSteer(_model::clear, x_nearestA._nearest.getState(), x_rand, timeForward);
        if (phiA == null)
            return 0;

        if (DEBUG)
            System.out.println(phiA);

        // now we have a clear trajectory from nearest to rand.

        double tMaxA = Math.max(phiA.x.s1.t + phiA.x.s2.t, phiA.y.s1.t + phiA.y.s2.t);
        Node<N4> freeEndA = null;

        if (curves) {
            // make lots of little segments
            double tStep = 0.1;
            Node<N4> source = x_nearestA._nearest;
            if (DEBUG)
                System.out.printf("phi %s\n", phiA);
            if (timeForward) {
                double tSoFar = 0;
                for (double tSec = tStep; tSec <= tMaxA; tSec += tStep) {
                    tSoFar = tSec;
                    Matrix<N4, N1> state = SampleTrajectory(phiA, tSec);
                    if (DEBUG)
                        System.out.printf("A1 stepstate %s\n", state);
                    Node<N4> target = new Node<>(state);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                    InsertNode(randLink, _T_a);
                    source = target;
                }
                if (tSoFar < tMaxA) {
                    // add one more segment to actually reach xrand
                    Node<N4> target = new Node<>(x_rand);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                    InsertNode(randLink, _T_a);
                }
            } else {
                // time is reversed, so walk the trajectory backwards
                double tSoFar = 0;
                for (double tSec = tMaxA; tSec >= 0; tSec -= tStep) {
                    tSoFar = tSec;
                    Matrix<N4, N1> state = SampleTrajectory(phiA, tSec);
                    if (DEBUG)
                        System.out.printf("A2 stepstate %s\n", state);
                    Node<N4> target = new Node<>(state);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                    InsertNode(randLink, _T_a);
                    source = target;
                }
                if (tSoFar > 0) {
                    // add one more segment to actually reach xrand
                    Node<N4> target = new Node<>(x_rand);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                    InsertNode(randLink, _T_a);
                }
            }
        } else {
            // just make one segment
            // same for forward and reverse cases.
            Node<N4> target = new Node<>(x_rand);
            freeEndA = target;

            LocalLink<N4> randLink = new LocalLink<>(x_nearestA._nearest, target, tMaxA);
            InsertNode(randLink, _T_a);
        }

        // SwapTrees();

        edges += 1;

        if (BIDIRECTIONAL) {

            // now check for feasible paths to some node in the other tree.
            // note that the continuity requirement is not to match the state, it's to match
            // the time-reversed state, since the two trees have opposite time polarity.

            KDNearNode<Node<N4>> x_nearestB = BangBangNearest(x_rand, _T_b, !timeForward);
            if (x_nearestB == null) {
                SwapTrees();
                return 1;
            }

            Trajectory phiB = BangBangSteer(_model::clear, x_nearestB._nearest.getState(), x_rand, !timeForward);
            if (phiB == null) {
                SwapTrees();
                return 1;
            }
            double tMaxB = Math.max(phiB.x.s1.t + phiB.x.s2.t, phiB.y.s1.t + phiB.y.s2.t);

            Node<N4> freeEndB = null;
            if (curves) {
                // make lots of little segments
                double tStep = 0.1;
                Node<N4> source = x_nearestB._nearest;
                if (DEBUG)
                    System.out.printf("phiB %s\n", phiB);
                if (!timeForward) {
                    double tSoFar = 0;
                    for (double tSec = tStep; tSec <= tMaxB; tSec += tStep) {
                        // for (double tSec = 0; tSec <= tMaxB; tSec += tStep) {
                        tSoFar = tSec;
                        Node<N4> source1 = source;
                        Matrix<N4, N1> state = SampleTrajectory(phiB, tSec);
                        if (DEBUG)
                            System.out.printf("A3 stepstate %s\n", state);
                        Node<N4> target = new Node<>(state);
                        freeEndB = target;
                        LocalLink<N4> randLink = new LocalLink<>(source1, target, tStep);
                        InsertNode(randLink, _T_b);
                        source1 = target;
                        source = source1;
                    }
                    if (tSoFar < tMaxB) {
                        // add one more segment to actually reach xrand
                        Node<N4> target = new Node<>(x_rand);
                        freeEndB = target;
                        LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                        InsertNode(randLink, _T_b);
                    }
                } else {
                    // time is reversed, so walk the trajectory backwards
                    double tSoFar = 0;
                    for (double tSec = tMaxB; tSec >= 0; tSec -= tStep) {
                        tSoFar = tSec;
                        Matrix<N4, N1> state = SampleTrajectory(phiB, tSec);
                        if (DEBUG)
                            System.out.printf("A4 stepstate %s\n", state);
                        Node<N4> target = new Node<>(state);
                        freeEndB = target;
                        LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                        InsertNode(randLink, _T_b);
                        source = target;
                    }
                    if (tSoFar > 0) {
                        // add one more segment to actually reach xrand
                        if (DEBUG)
                            System.out.printf("A4 last state %s\n", x_rand);
                        Node<N4> target = new Node<>(x_rand);
                        freeEndB = target;
                        LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                        InsertNode(randLink, _T_b);
                    }
                }
            } else {
                // just make one segment
                // same for forward and reverse cases.
                Node<N4> target = new Node<>(x_rand);
                freeEndB = target;
                LocalLink<N4> randLink = new LocalLink<>(x_nearestB._nearest, target, tMaxB);
                InsertNode(randLink, _T_b);
            }

            // if we got here then we are finished constructing a path, and should stop
            // and start optimizing it.

            // for now make a "path"

            if (freeEndB != null) {
                Path<N4> p = GeneratePath(freeEndA, freeEndB);
                if (_sigma_best == null) {
                    System.out.printf("first path distance %7.3f\n", p.getDistance());
                    _sigma_best = p;
                } else {
                    if (p.getDistance() < _sigma_best.getDistance()) {
                        System.out.printf("new best path distance %7.3f\n", p.getDistance());
                        _sigma_best = p;
                    }
                }
                // TODO replace above with this
                SinglePath<N4> sp = GenerateSinglePath(freeEndA, freeEndB);
                if (_single_sigma_best == null) {
                    System.out.printf("first path distance %7.3f\n", sp.getDistance());
                    _single_sigma_best = sp;
                } else {
                    if (sp.getDistance() < _single_sigma_best.getDistance()) {
                        System.out.printf("new best path distance %7.3f\n", sp.getDistance());
                        _single_sigma_best = sp;
                    }
                }

                // bail so that we can stop looking
                return -1;
            }
            // if (DEBUG)
            // System.out.println("PATH " + p);
            // if (_sigma_best == null) {
            // System.out.printf("first path distance %7.3f\n", p.getDistance());
            // _sigma_best = p;
            // } else {
            // if (p.getDistance() < _sigma_best.getDistance()) {
            // System.out.printf("new best path distance %7.3f\n", p.getDistance());
            // _sigma_best = p;
            // }

            // LocalLink<N4> randLink = new LocalLink<>(x_nearest._nearest, new
            // Node<>(x_rand), tMax);

            // LocalLink<N4> randLink = SampleFree(timeForward);
            // if (DEBUG)
            // System.out.println("randLink: " + randLink);

            // if (CollisionFree(randLink.get_source().getState(),
            // randLink.get_target().getState())) {
            // new node in "this" tree

            // Node<N4> newNode = InsertNode(randLink, _T_a);

            // if (DEBUG)
            // System.out.println("NEW NODE " + newNode);

            // List<NearNode<N4>> X_nearA = Near(newNode.getState(), _T_a);
            // Rewire(X_nearA, newNode, timeForward);

            // // is there a point in the other tree that is reachable from
            // // the node we just inserted? reachable nodes are nearby in a Euclidean
            // // sense, though most Euclidean nearby nodes are not reachable.
            // // so start with a list of near nodes and test them one by one.

            // // near nodes in the other tree:
            // List<NearNode<N4>> X_near = Near(newNode.getState(), _T_b);
            // Matrix<N4, N1> x1 = newNode.getState();
            // for (NearNode<N4> nearNode : X_near) {
            // // one near node in the other tree
            // Matrix<N4, N1> x2 = nearNode.node.getState();

            // ShootingSolver<N4, N2>.Solution sol = solver.solve(Nat.N4(), Nat.N2(), f, x1,
            // x2, timeForward);
            // if (sol != null) {
            // // there's a route from x1 aka newnode (in a) to x2 aka nearnode (in b)
            // if (DEBUG)
            // System.out.printf("FOUND feasible link x1: %s x2: %s sol: %s\n",
            // Util.matStr(x1), Util.matStr(x2), sol);
            // // TODO: do something with the solution u value
            // // add a node in a corresponding to the near node in b
            // LocalLink<N4> newInA = new LocalLink<>(newNode, new
            // Node<>(nearNode.node.getState()),
            // Math.abs(sol.dt));
            // Node<N4> newNewNode = InsertNode(newInA, _T_a);
            // Rewire(X_near, newNewNode, timeForward);
            // // create a path that traverses the new link.
            // Path<N4> p = GeneratePath(newNewNode, nearNode.node);
            // if (DEBUG)
            // System.out.println("PATH " + p);
            // if (_sigma_best == null) {
            // System.out.printf("first path distance %7.3f\n", p.getDistance());
            // _sigma_best = p;
            // } else {
            // if (p.getDistance() < _sigma_best.getDistance()) {
            // System.out.printf("new best path distance %7.3f\n", p.getDistance());
            // _sigma_best = p;
            // }
            // }
            // // don't need more than one feasible link
            // break;
            // }
            // }
            // }
            SwapTrees();
        }
        return edges;
    }

    public void SwapTrees() {
        KDNode<Node<N4>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    public void Optimize() {
        SinglePath<N4> singlePath = _single_sigma_best;

        // List<Matrix<N4, N1>> states = singlePath.getStates();
        List<SinglePath.Link<N4>> links = singlePath.getLinks();

        int nodect = links.size();
        int node1 = random.nextInt(nodect);
        int node2 = random.nextInt(nodect);
        // note that node1 and node2 could be the same
        if (node1 > node2) {
            int tmp = node1;
            node1 = node2;
            node2 = tmp;
        }
        // now node1 is first
        // actually we want the sub-list
        List<SinglePath.Link<N4>> sublist = links.subList(node1, node2 + 1);
        // these could be the same, just replace a single link.
        Matrix<N4, N1> state1 = sublist.get(0).x_i;
        Matrix<N4, N1> state2 = sublist.get(sublist.size() - 1).x_g;
        double cost = 0;
        for (SinglePath.Link<N4> link : sublist) {
            cost += link.cost;
        }

        // try to get there

        Trajectory phiA = BangBangSteer(_model::clear, state1, state2, true);
        if (phiA == null)
            return;

        double tMaxA = Math.max(phiA.x.s1.t + phiA.x.s2.t, phiA.y.s1.t + phiA.y.s2.t);

        // double tOptimal = tOptimal(state1, state2, MAX_U);
        if (tMaxA >= cost)
            return;

        // we can do better
        List<SinglePath.Link<N4>> replacement = new ArrayList<>();

        double tStep = 0.1;

        double tSoFar = 0;
        for (double tSec = tStep; tSec <= tMaxA; tSec += tStep) {
            tSoFar = tSec;
            Matrix<N4, N1> state = SampleTrajectory(phiA, tSec);
            SinglePath.Link<N4> randLink = new SinglePath.Link<>(state1, state, tStep);
            replacement.add(randLink);
            state1 = state;
        }
        if (tSoFar < tMaxA) {
            // add one more segment to actually reach xrand
            SinglePath.Link<N4> randLink = new SinglePath.Link<>(state1, state2, tStep);
            replacement.add(randLink);
        }

        sublist.clear();
        links.addAll(node1, replacement);
        _single_sigma_best = new SinglePath<>(links);

    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     * 
     * @param x_1 Node in one tree
     * @param x_2 Same state in the other tree
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

    /**
     * Returns a copy of the solution path.
     * Param nodes need to have the same state.
     * 
     * @param x_1 "leaf" end of one of the trees; could be initial or goal depending
     *            on swapping
     * @param x_2 leaf of the other tree
     */
    SinglePath<N4> GenerateSinglePath(Node<N4> x_1, Node<N4> x_2) {
        if (!x_1.getState().isEqual(x_2.getState(), 0.001))
            throw new IllegalArgumentException(
                    "x1 " + x_1.getState().toString() + " != x2 " + x_2.getState().toString());

        // the list of states and links returned starts at the leaf and ends at the
        // root.
        SinglePath<N4> p_1 = walkParentsSingle(new HashSet<>(), x_1);
        if (DEBUG)
            System.out.println("p1 " + p_1);
        SinglePath<N4> p_2 = walkParentsSingle(new HashSet<>(), x_2);
        if (DEBUG)
            System.out.println("p2 " + p_2);
        // either p_1 or p_2 are the initial tree
        //
        // boolean root1 = same(_T_a.getValue().getState(), p_1.getRoot());
        boolean root1 = same(_T_a.getValue().getState(), p_1.getFirstLink().x_i);
        if (!root1) {
            // swap them
            SinglePath<N4> tmp = p_1;
            p_1 = p_2;
            p_2 = tmp;
        }

        // now p_1 root is the initial state.

        // new: reverse the other one since the walker doesn't do it.

        // List<Matrix<N4, N1>> states_1 = p_1.getStates();
        // Collections.reverse(states_1);

        // List<Matrix<N4, N1>> states_2 = p_2.getStates();
        // don't include the same point twice
        // states_2.remove(0);

        // List<Matrix<N4, N1>> result = new ArrayList<>();
        // result.addAll(states_1);
        // result.addAll(states_2);

        // this list is from leaf to root, so backwards. reverse it.
        List<SinglePath.Link<N4>> links1 = p_1.getLinks();
        LinkedList<SinglePath.Link<N4>> revLinks1 = new LinkedList<>();
        for (SinglePath.Link<N4> l : links1) {
            revLinks1.addFirst(new SinglePath.Link<>(l.x_g, l.x_i, l.cost));
        }
        // this list is in the correct order.
        List<SinglePath.Link<N4>> links2 = p_2.getLinks();

        List<SinglePath.Link<N4>> resultLinks = new ArrayList<>();
        resultLinks.addAll(revLinks1);
        resultLinks.addAll(links2);

        return new SinglePath<>(/* result, */ resultLinks);
    }

    boolean CollisionFree(Matrix<N4, N1> from, Matrix<N4, N1> to) {
        return _model.link(from, to);
    }

    /** Sample the free state. */
    Matrix<N4, N1> SampleState() {
        // if (random.nextDouble() > 0.95) return _model.goal();
        while (true) {
            Matrix<N4, N1> newConfig = _sample.get();
            if (_model.clear(newConfig))
                return newConfig;
        }
    }

    /**
     * Return the nearest node in the tree.
     * 
     * 1. find a set of Euclidean-near nodes from the KD tree, using a kinda
     * arbitrary radius.
     * 2. compute the optimal (fastest coordinated) time from xInitial to each node,
     * using the tOptimal function. in time-reversed mode the initial and final
     * nodes are swapped.
     * 
     * @param xNew     the goal state (x xdot y ydot)
     * @param rootNode the tree to look through
     */
    KDNearNode<Node<N4>> BangBangNearest(Matrix<N4, N1> xNew, KDNode<Node<N4>> rootNode, boolean timeForward) {
        // For now, use the Near function, which uses the "radius". Maybe
        // it would be better to choose top-N-near, or use a different radius,
        // or whatever.
        ArrayList<NearNode<N4>> nodes = Near(xNew, rootNode);
        double tMin = Double.MAX_VALUE;
        Node<N4> bestNode = null;
        for (NearNode<N4> node : nodes) {
            // rescore each node.
            double tOptimal;
            if (timeForward) {
                tOptimal = tOptimal(node.node.getState(), xNew, MAX_U);
            } else {
                tOptimal = tOptimal(xNew, node.node.getState(), MAX_U);
            }
            if (tOptimal < tMin) {
                tMin = tOptimal;
                bestNode = node.node;
            }
        }
        if (tMin == Double.MAX_VALUE)
            return null;

        return new KDNearNode<>(tMin, bestNode);
    }

    /**
     * Compute the optimal (fastest coordinated) time from x_i to x_g. For time
     * reversal, the caller should swap the arguments.
     * 
     * Each axis is solved separately, and then coordinated by slowing the faster
     * axis, while respecting the "gap" identified by LaSalle et al [1], see
     * proposition 1.
     *
     * States are (x, xdot, y, ydot)
     * 
     * @param x_i  initial state
     * @param x_g  goal state
     * @param umax
     */
    static double tOptimal(
            Matrix<N4, N1> x_i,
            Matrix<N4, N1> x_g,
            // boolean timeForward,
            double umax) {
        // Matrix<N4, N1> x_i;
        // Matrix<N4, N1> x_g;
        // if (timeForward) {
        // x_i = x1;
        // x_g = x2;
        // } else {
        // x_i = x2;
        // x_g = x1;
        // }
        // TODO: handle time reversal

        double xTSwitch = tSwitch(
                x_i.get(0, 0),
                x_i.get(1, 0),
                x_g.get(0, 0),
                x_g.get(1, 0),
                umax);
        double yTSwitch = tSwitch(
                x_i.get(2, 0),
                x_i.get(3, 0),
                x_g.get(2, 0),
                x_g.get(3, 0),
                umax);
        double xTLimit = tLimit(
                x_i.get(0, 0),
                x_i.get(1, 0),
                x_g.get(0, 0),
                x_g.get(1, 0),
                umax);
        double yTLimit = tLimit(
                x_i.get(2, 0),
                x_i.get(3, 0),
                x_g.get(2, 0),
                x_g.get(3, 0),
                umax);
        double xTMirror = tMirror(
                x_i.get(0, 0),
                x_i.get(1, 0),
                x_g.get(0, 0),
                x_g.get(1, 0),
                umax);
        double yTMirror = tMirror(
                x_i.get(2, 0),
                x_i.get(3, 0),
                x_g.get(2, 0),
                x_g.get(3, 0),
                umax);

        if (DEBUG)
            System.out.printf("xs %5.3f xl %5.3f xm %5.3f ys %5.3f yl %5.3f ym %5.3f\n",
                    xTSwitch, xTLimit, xTMirror, yTSwitch, yTLimit, yTMirror);

        List<Item> opts = new ArrayList<>();
        put(opts, xTSwitch, Solution.SWITCH);
        put(opts, yTSwitch, Solution.SWITCH);
        put(opts, xTLimit, Solution.LIMIT);
        put(opts, yTLimit, Solution.LIMIT);
        put(opts, xTMirror, Solution.MIRROR);
        put(opts, yTMirror, Solution.MIRROR);
        Collections.sort(opts);

        int solved = 0;
        for (Item item : opts) {
            switch (item.s) {
                case SWITCH:
                    ++solved;
                    break;
                case LIMIT:
                    --solved;
                    break;
                case MIRROR:
                    ++solved;
                    break;
            }
            if (solved == 2) {
                if (DEBUG)
                    System.out.printf("returning t %5.3f\n", item.t);
                return item.t;
            }
        }
        // this should never happen; there is never not a solution.
        throw new IllegalArgumentException(String.format("%s\n%s\nx %f %f %f\ny %f %f %f\n",
                x_i.toString(), x_g.toString(), xTSwitch, xTLimit, xTMirror, yTSwitch, yTLimit, yTMirror));
    }

    static void put(List<Item> opts, double t, Solution solution) {
        if (!Double.isNaN(t) && t >= 0)
            opts.add(new Item(t, solution));
    }

    static class Item implements Comparable<Item> {
        double t;
        Solution s;

        public Item(double t, Solution s) {
            this.t = t;
            this.s = s;
        }

        @Override
        public int compareTo(Item arg0) {
            return Double.compare(t, arg0.t);
        }

        @Override
        public int hashCode() {
            final int prime = 31;
            int result = 1;
            long temp;
            temp = Double.doubleToLongBits(t);
            result = prime * result + (int) (temp ^ (temp >>> 32));
            result = prime * result + ((s == null) ? 0 : s.hashCode());
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Item other = (Item) obj;
            if (Double.doubleToLongBits(t) != Double.doubleToLongBits(other.t))
                return false;
            if (s != other.s)
                return false;
            return true;
        }

        @Override
        public String toString() {
            return "Item [t=" + t + ", s=" + s + "]";
        }
    }

    /**
     * The method of LaSalle et al [1] of "steering" is to follow the trajectory
     * until it hits something, but doing that also implies redoing the trajectory
     * with a resting state at the obstacle boundary, which results in lots of
     * trajectories with weird corners at boundaries, which seems like a bad thing
     * to encourage, even though they can be later optimized away.
     * 
     * So instead i'll just check for collisions, return the trajectory if there
     * aren't any, and otherwise return null.
     * 
     * so we need a way to sample the trajectory.
     * 
     * @param free        predicate indicating collision-free
     * @param x_i         initial state
     * @param x_g         goal state
     * @param timeForward
     * @return a feasible trajectory from initial to goal, or null if none is
     *         feasible.
     */
    static Trajectory BangBangSteer(
            Predicate<Matrix<N4, N1>> free,
            Matrix<N4, N1> x_i,
            Matrix<N4, N1> x_g,
            boolean timeForward) {
        Trajectory trajectory;
        if (timeForward) {
            trajectory = optimalTrajectory(x_i, x_g, MAX_U);
        } else {
            trajectory = optimalTrajectory(x_g, x_i, MAX_U);
        }
        if (trajectory == null)
            return null;
        double tMax = Math.max(trajectory.x.s1.t + trajectory.x.s2.t, trajectory.y.s1.t + trajectory.y.s2.t);
        double tStep = 0.1;
        for (double tSec = 0; tSec < tMax; tSec += tStep) {
            Matrix<N4, N1> state = SampleTrajectory(trajectory, tSec);
            if (!free.test(state))
                return null;
        }
        return trajectory;
    }

    /**
     * Since the trajectories are constant-acceleration, sampling is simple.
     * Hauser's code yields spatial coordinates only, which i guess is all you need
     * for collision checking, but i kinda want to see them all.
     */
    static Matrix<N4, N1> SampleTrajectory(Trajectory t, double tSec) {
        Matrix<N2, N1> xSample = SampleAxis(t.x, tSec);
        Matrix<N2, N1> ySample = SampleAxis(t.y, tSec);
        return new Matrix<>(Nat.N4(), Nat.N1(), new double[] {
                xSample.get(0, 0), xSample.get(1, 0),
                ySample.get(0, 0), ySample.get(1, 0),
        });
    }

    static Matrix<N2, N1> SampleAxis(Trajectory.Axis a, double tSec) {
        double timeTotal = a.s1.t + a.s2.t;
        if (tSec < 0) {
            return VecBuilder.fill(a.i, a.idot);
        } else if (tSec > timeTotal) {
            return VecBuilder.fill(a.g, a.gdot);
        } else if (Math.abs(tSec) < 1e-6) {
            return VecBuilder.fill(a.i, a.idot);
        } else if (Math.abs(tSec - timeTotal) < 1e-6) {
            return VecBuilder.fill(a.g, a.gdot);
        } else if (tSec < a.s1.t) {
            // first segment
            double x = a.i + a.idot * tSec + 0.5 * a.s1.u * tSec * tSec;
            double xdot = a.idot + a.s1.u * tSec;
            return VecBuilder.fill(x, xdot);
        } else {
            double timeToGo = timeTotal - tSec; // a positive number
            double x = a.g - a.gdot * timeToGo + 0.5 * a.s2.u * timeToGo * timeToGo;
            double xdot = a.gdot - a.s2.u * timeToGo;
            return VecBuilder.fill(x, xdot);
        }
    }

    public enum Solution {
        SWITCH, LIMIT, MIRROR
    };

    public static class Trajectory {
        public static class Axis {
            public static class Segment {
                double u;
                double t;

                @Override
                public String toString() {
                    return "Segment [u=" + u + ", t=" + t + "]";
                }
            }

            double i; // initial position
            double idot; // initial velocity
            double g; // goal position
            double gdot; // goal velocity
            Segment s1 = new Segment();
            Segment s2 = new Segment();

            @Override
            public String toString() {
                return "Axis [i=" + i + ", idot=" + idot + ", g=" + g + ", gdot=" + gdot + ",\n s1=" + s1 + ",\n s2="
                        + s2
                        + "]";
            }
        }

        Axis x = new Axis();
        Axis y = new Axis();

        @Override
        public String toString() {
            return "Trajectory [\nx=" + x + ",\n y=" + y + "]";
        }
    }

    Matrix<N4, N1> sample(Trajectory traj, double t) {
        return null;
    }

    Trajectory connect(Matrix<N4, N1> start, Matrix<N4, N1> end) {
        return null;
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
            // TODO: this is actually a cylindrical space, so make it so
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
     * not literally to the right of initial state, but to the right of the
     * switching surface containing the initial state. the paper has a different
     * way of expressing this which i think might be wrong.
     * https://arxiv.org/pdf/2210.01744.pdf
     * 
     * The params are primitives because we kinda pick them out of the N4 state.
     * 
     * @param i    initial position
     * @param idot initial velocity
     * @param g    goal position
     * @param gdot goal velocity
     */
    public static boolean goalRight(double i, double idot, double g, double gdot, double umax) {
        // intercept of umax parabola intersecting x1
        double c_I_plus = c_plus(i, idot, umax);
        // intercept of umin parabola intersecting x1
        double c_I_minus = c_minus(i, idot, umax);
        // intercept of umax parabola intersecting x2
        double c_G_plus = c_plus(g, gdot, umax);
        // intercept of umin parabola intersecting x2
        double c_G_minus = c_minus(g, gdot, umax);

        if (gdot > idot) {
            if (c_G_plus > c_I_plus) {
                return true;
            }
            return false;
        }
        if (c_G_minus > c_I_minus) {
            return true;
        }
        return false;
    }

    /**
     * for a single 2d double-integrator, this is the time
     * to traverse x_i x_switch x_g.
     * 
     * there are three possible paths, of which one is sure to exist.
     * 
     * through x_switch
     * through x_limit
     * through x_mirror
     * 
     * this function returns the x_switch version.
     * 
     * TODO: return both t1 and t2
     * TODO: something about waiting time? i think this all might be wrong.
     */
    public static double tSwitch(double i, double idot, double g, double gdot, double umax) {
        // if (RRTStar7.goalRight(i, idot, g, gdot, umax)) {

        // these might return NaN for impossible paths

        double tIplusGminus = tSwitchIplusGminus(i, idot, g, gdot, umax);
        double tIminusGplus = tSwitchIminusGplus(i, idot, g, gdot, umax);

        // it's also possible for more than one path to be returned, one
        // faster than the other.

        if (Double.isNaN(tIplusGminus) || tIplusGminus > 1e100 || tIplusGminus < 0) {
            // there should be at least one solution
            if (Double.isNaN(tIminusGplus) || tIminusGplus > 1e100 || tIminusGplus < 0) {
                throw new IllegalArgumentException(String.format("A %f %f %f %f %f %f %f",
                        tIplusGminus, tIminusGplus, i, idot, g, gdot, umax));
            }
            return tIminusGplus;

        }
        if (Double.isNaN(tIminusGplus) || tIminusGplus > 1e100 || tIminusGplus < 0) {
            return tIplusGminus;
        }
        // if we got here, then they're both actually numbers.
        // so just return the slower one for now.
        // TODO: be clever about waiting time etc.

        // as specified in the paper (p2), we choose the *faster* of the
        // feasible paths as the tSwitch path.
        // we'll use the slower one as tLimit, later.

        return Math.min(tIplusGminus, tIminusGplus);
    }

    /**
     * time to traverse x_i x_switch x_g.
     * 
     * This is for the I+G- path.
     */
    static double tSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        double q_dot_switch = qDotSwitchIplusGminus(i, idot, g, gdot, umax);
        // time from initial to switching point
        double t_1 = (q_dot_switch - idot) / umax;
        // time from switching to final, note i think this is a mistake
        // in the paper.
        double t_2 = (gdot - q_dot_switch) / (-1.0 * umax);
        if (DEBUG)
            System.out.printf("I+G- qdotsw %5.3f t1 %5.3f t2 %5.3f\n", q_dot_switch, t_1, t_2);
        // these weird comparisons are because sometimes there is "negative zero"
        if (t_1 < -0.001 || t_2 < -0.001)
            return Double.NaN;
        return t_1 + t_2;
    }

    /**
     * Time to traverse x_i x_switch x_g.
     * 
     * This is for the I-G+ path.
     * 
     * for this path,
     */
    static double tSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        double q_dot_switch = qDotSwitchIminusGplus(i, idot, g, gdot, umax);
        // time from initial to switching point
        double t_1 = (q_dot_switch - idot) / (-1.0 * umax);
        // time from switching to final, note i think this is a mistake
        // in the paper.
        double t_2 = (gdot - q_dot_switch) / umax;
        if (DEBUG)
            System.out.printf("I-G+ qdotsw %5.3f t1 %5.3f t2 %5.3f\n", q_dot_switch, t_1, t_2);
        if (t_1 < -0.001 || t_2 < -0.001)
            return Double.NaN;
        return t_1 + t_2;
    }

    /**
     * Position at x_switch, which is the midpoint between the curves
     * intersecting the initial and goal states.
     * 
     * This is for the I+G- path.
     */
    static double qSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        return (c_plus(i, idot, umax) + c_minus(g, gdot, umax)) / 2;
    }

    /**
     * Position at x_switch, which is the midpoint between the curves
     * intersecting the initial and goal states.
     * 
     * This is for the I-G+ path.
     */
    static double qSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        return (c_minus(i, idot, umax) + c_plus(g, gdot, umax)) / 2;
    }

    /**
     * Velocity at x_switch. this should be faster than idot.
     * 
     * This is for the I+G- path.
     * 
     * For this path qDotSwitch is always positive; otherwise it's an x_limit path.
     */
    static double qDotSwitchIplusGminus(double i, double idot, double g, double gdot, double umax) {
        return Math.sqrt(
                2 * umax * (qSwitchIplusGminus(i, idot, g, gdot, umax) - c_plus(i, idot, umax)));
    }

    /**
     * Velocity at x_switch. this should be faster than idot.
     * 
     * This is for the I-G+ path.
     * 
     * For this path qDotSwitch is always negative, otherwise it's an x_switch path.
     * 
     * Note the intersection here may imply negative-time traversal of one of the
     * segments.
     */
    static double qDotSwitchIminusGplus(double i, double idot, double g, double gdot, double umax) {
        return -1.0 * Math.sqrt(
                2 * umax * (qSwitchIminusGplus(i, idot, g, gdot, umax) - c_plus(g, gdot, umax)));
    }

    /**
     * Velocity at x_limit. this should be slower than idot.
     * 
     * This is for the I+G- path.
     * 
     * For this path qDotLimit is always negative; otherwise it's an x_switch path.
     */
    static double qDotLimitIplusGminus(double i, double idot, double g, double gdot, double umax) {
        return -1.0 * Math.sqrt(
                2 * umax * (qSwitchIplusGminus(i, idot, g, gdot, umax) - c_plus(i, idot, umax)));
    }

    /**
     * Velocity at x_limit. this should be slower than idot.
     * 
     * This is for the I-G+ path.
     * 
     * For this path qDotLimit is always positive; otherwise it's an x_switch path.
     */
    static double qDotLimitIminusGplus(double i, double idot, double g, double gdot, double umax) {
        return Math.sqrt(
                2 * umax * (qSwitchIminusGplus(i, idot, g, gdot, umax) - c_plus(g, gdot, umax)));
    }

    /** Intercept of negative-U path intersecting the state */
    static double c_minus(double x, double xdot, double umax) {
        return x - Math.pow(xdot, 2) / (-2.0 * umax);
    }

    /** Intercept of positive-U path intersecting the state */
    static double c_plus(double x, double xdot, double umax) {
        return x - Math.pow(xdot, 2) / (2.0 * umax);
    }

    /**
     * returns the slowest path possible without crossing the axis
     * 
     * the difference between tLimit and tMirror is the "gap".
     */
    static double tLimit(double i, double idot, double g, double gdot, double umax) {
        double qDotLimitIplusGminus = qDotLimitIplusGminus(i, idot, g, gdot, umax);
        double qDotLimitIminusGplus = qDotLimitIminusGplus(i, idot, g, gdot, umax);
        if (Double.isNaN(qDotLimitIplusGminus) || Double.isNaN(qDotLimitIminusGplus))
            return Double.NaN;

        // the slow path involves the smaller of the two qDot values
        if (Math.abs(qDotLimitIplusGminus) > Math.abs(qDotLimitIminusGplus)) {
            double qDotLimit = qDotLimitIminusGplus;
            double t_1 = (qDotLimit - idot) / (-1.0 * umax);
            double t_2 = (gdot - qDotLimit) / umax;
            if (t_1 < 0 || t_2 < 0)
                return Double.NaN;
            return t_1 + t_2;
        }
        double qDotLimit = qDotLimitIplusGminus;
        double t_1 = (qDotLimit - idot) / umax;
        double t_2 = (gdot - qDotLimit) / (-1.0 * umax);
        if (t_1 < 0 || t_2 < 0)
            return Double.NaN;
        return t_1 + t_2;
    }

    /**
     * returns the slowest path that crosses the axis.
     * 
     * the difference between tLimit and tMirror is the "gap".
     */
    static double tMirror(double i, double idot, double g, double gdot, double umax) {
        double tLimit = tLimit(i, idot, g, gdot, umax);
        if (Double.isNaN(tLimit))
            return Double.NaN;

        double qDotLimitIplusGminus = qDotLimitIplusGminus(i, idot, g, gdot, umax);
        double qDotLimitIminusGplus = qDotLimitIminusGplus(i, idot, g, gdot, umax);

        // use the slow one
        if (Math.abs(qDotLimitIplusGminus) > Math.abs(qDotLimitIminusGplus)) {
            double qDotLimit = qDotLimitIminusGplus;
            double t_1 = 2.0 * qDotLimit / umax;
            double t_2 = -2.0 * qDotLimit / (-1.0 * umax);
            if (t_1 < 0 || t_2 < 0)
                return Double.NaN;
            return tLimit + t_1 + t_2;
        }
        double qDotLimit = qDotLimitIplusGminus;
        double t_1 = 2.0 * qDotLimit / (-1.0 * umax);
        double t_2 = -2.0 * qDotLimit / umax;
        if (t_1 < 0 || t_2 < 0)
            return Double.NaN;
        return tLimit + t_1 + t_2;
    }

    /**
     * Return a 4d trajectory that moves from x_i to x_g with the umax constraint,
     * completing both axes at the same time.
     * 
     * for time reversal, the caller should swap the arguments.
     * 
     * returns null if failure.
     */
    static Trajectory optimalTrajectory(Matrix<N4, N1> x_i, Matrix<N4, N1> x_g, double umax) {
        double tOptimal = tOptimal(x_i, x_g, umax);
        Trajectory result = new Trajectory();
        result.x = slowU(x_i.get(0, 0), x_i.get(1, 0), x_g.get(0, 0), x_g.get(1, 0), tOptimal);
        result.y = slowU(x_i.get(2, 0), x_i.get(3, 0), x_g.get(2, 0), x_g.get(3, 0), tOptimal);
        if (result.x == null || result.y == null)
            return null;
        return result;
    }

    /**
     * This is the method of Hauser et al [1] (see section D) for the
     * parabola-linear-parabola solution, i.e. bang bang with a maximum speed.
     * This is from ParabolicRamp.cpp.
     */
    static Trajectory.Axis PLP(double i, double idot, double g, double gdot, double tw, double vmax) {
        double a1 = aPLP(i, idot, g, gdot, tw, vmax);
        double a2 = aPLP(i, idot, g, gdot, tw, -1.0 * vmax);
        double a = Double.MAX_VALUE;
        double v = Double.MAX_VALUE;
        double tswitch1 = 0;
        double tswitch2 = 0;
        double ttotal = 0;
        if (Math.abs(a1) < a) {
            a = a1;
            v = vmax;
        }
        if (Math.abs(a2) < a) {
            a = a2;
            v = -1.0 * vmax;
        }
        if (a > 1e100) {
            return null;
        }
        if (Math.abs(a) < 0.001) {
            tswitch1 = 0;
            tswitch2 = tw;
            ttotal = tw;
        } else {
            ttotal = CalcTotalTime(i, idot, g, gdot, a, v);
            tswitch1 = CalcSwitchTime1(i, idot, g, gdot, a, v);
            tswitch2 = CalcSwitchTime2(i, idot, g, gdot, a, v);
        }
        return null;
    }

    /**
     * This is from ParabolicRamp.cpp.
     */
    static double CalcTotalTime(double i, double idot, double g, double gdot, double a, double v) {
        double t1 = (v - idot) / a;
        double t2mt = (gdot - v) / a;
        double t2mt1 = 0.5 * (gdot * gdot + idot * idot) / (v * a) - (v / a) + (g - i) / v;
        if (t1 < 0 || t2mt > 0 || t2mt1 < 0)
            return Double.NaN;
        if (t1 > 1e100 || t2mt > 1e100)
            return Double.NaN;
        return t1 + t2mt1 - t2mt;
    }

    /**
     * This is from ParabolicRamp.cpp.
     */
    static double CalcSwitchTime1(double i, double idot, double g, double gdot, double a, double v) {
        double t1 = (v - idot) / a;
        if (t1 < 0)
            return Double.NaN;
        return t1;
    }

    /**
     * This is from ParabolicRamp.cpp.
     */
    static double CalcSwitchTime2(double i, double idot, double g, double gdot, double a, double v) {
        double t1 = (v - idot) / a;
        double y1 = 0.5 * (v * v - idot * idot) / a + i;
        double y2 = 0.5 * (gdot * gdot - v * v) / a + g;
        System.out.printf("y1 %f y2 %f\n", y1, y2);
        double t2mt1 = (y2 - y1) / v;
        // double t2mt1 = 0.5 * (gdot * gdot + idot * idot) / (v * a) - (v / a) + (g -
        // i) / v;
        if (t1 < 0 || t2mt1 < 0)
            return Double.NaN;
        return t1 + t2mt1;
    }

    /**
     * this is from ParabolicRamp.cpp:CalcMinAcell, which has some extra code to
     * deal with values near zero, etc
     * TODO: add that?
     */
    static double aPLP(double i, double idot, double g, double gdot, double tw, double vmax) {
        // return (vmax * vmax - vmax * (idot + gdot) + 0.5 * (idot * idot + gdot *
        // gdot)) / (tw * vmax - (g - i));

        double den = tw - (g - i) / vmax;
        double num = vmax * 0.5 * (idot / vmax - 1.0) * (idot / vmax - 1.0) + (gdot / vmax - 1.0) * (gdot / vmax - 1.0);
        return num / den;
    }

    /**
     * This is the method of Hauser et al [1] (see section D) to solve the two-point
     * boundary problem for a single double integrator, minimizing the control
     * output (i.e. current or acceleration). another attempt to find a for fixed T
     * so the paper says solve the quadratic
     * T^2a^2 + sigma(2T(idot+gdot) + 4(i-g))a - (gdot-idot)^2 = 0.
     * 
     * Note that the ParabolicRamp code works differently from this
     * TODO: try that method
     * 
     * returns a two-part trajectory for one axis, or null if it fails.
     */
    static Trajectory.Axis slowU(double i, double idot, double g, double gdot, double tw) {
        double a = tw * tw;
        double b = 2.0 * tw * (idot + gdot) + 4.0 * (i - g);
        double c = -1.0 * (gdot - idot) * (gdot - idot);
        if (DEBUG)
            System.out.printf("a %f b %f c %f\n", a, b, c);

        // I+G-
        List<Double> plus = quadratic(a, b, c);
        // I-G+
        List<Double> minus = quadratic(a, -b, c);
        if (DEBUG)
            System.out.printf("plus %s minus %s\n", plus, minus);
        // we generally want the *lowest* acceleration that will solve the problem
        Double aMin = null;
        Trajectory.Axis result = new Trajectory.Axis();
        for (Double p : plus) {
            if (Math.abs(p) < 1e-6 && plus.size() > 1) {
                // zero is only ok if it's the only solution
                if (DEBUG)
                    System.out.println("reject p = 0 with two solutions");
                continue;
            }
            double ts;
            if (Math.abs(p) < 1e-6) {
                // if there is a zero solution then it runs the whole time
                ts = tw;
            } else {
                ts = 0.5 * (tw + (gdot - idot) / p);
            }
            if (DEBUG)
                System.out.printf("p %f ts %f\n", p, ts);
            if (p < 0) {
                if (DEBUG)
                    System.out.println("reject p < 0");
                continue;
            }
            if (ts < 0) {
                if (DEBUG)
                    System.out.println("reject ts < 0");
                continue;
            }

            if (ts > tw) {
                // switching time can't be more than total time
                if (DEBUG)
                    System.out.println("reject ts > tw");
                continue;

            }
            if (aMin == null || p < aMin) {
                if (DEBUG)
                    System.out.printf("accept p %f\n", p);
                aMin = p;
                result.i = i;
                result.idot = idot;
                result.g = g;
                result.gdot = gdot;
                result.s1.u = aMin; // I+G-
                result.s1.t = ts;
                result.s2.u = -aMin;
                result.s2.t = tw - ts;
            }
        }
        for (Double m : minus) {
            if (Math.abs(m) < 1e-6 && minus.size() > 1) {
                // zero is only ok if it's the only solution
                if (DEBUG)
                    System.out.println("reject p = 0 with two solutions");
                continue;
            }
            double ts;
            if (Math.abs(m) < 1e-6) {
                ts = tw;
            } else {
                // ts = 0.5 * (tw + (gdot - idot) / m);
                // is this right? switching i and g for minus?
                ts = 0.5 * (tw + (idot - gdot) / m);
            }
            if (DEBUG)
                System.out.printf("m %f ts %f\n", m, ts);
            if (m < 0) {
                if (DEBUG)
                    System.out.println("reject m < 0");
                continue;
            }
            if (ts < 0) {
                if (DEBUG)
                    System.out.println("reject ts < 0");
                continue;
            }
            if (ts > tw) {
                if (DEBUG)
                    System.out.println("reject ts > tw");
                continue;
            }
            if (aMin == null || m < aMin) {
                if (DEBUG)
                    System.out.printf("accept m %f\n", m);
                aMin = m;
                result.i = i;
                result.idot = idot;
                result.g = g;
                result.gdot = gdot;
                result.s1.u = -aMin; // I-G+
                result.s1.t = ts;
                result.s2.u = aMin;
                result.s2.t = tw - ts;
            }
        }
        if (aMin == null)
            return null;
        return result;
    }

    /**
     * returns roots of the quadratic
     * see KrisLibrary/planning/ParabolicRamp.cpp
     */
    static List<Double> quadratic(double a, double b, double c) {
        if (a == 0) {
            if (b == 0) {
                if (c == 0)
                    return List.of();
                return List.of();
            }
            return List.of(-c / b);

        }
        if (c == 0) { // det = b^2
            if (b == 0) // just y=ax^2
                return List.of(0.0);
            return List.of(0.0, -b / a);
        }
        double det = b * b - 4.0 * a * c;
        if (det < 0.0)
            return List.of();
        if (det == 0.0) {
            return List.of(-b / (2.0 * a));
        }
        det = sqrt(det);
        double x1;
        double x2;
        if (Math.abs(-b - det) < Math.abs(a))
            x1 = 0.5 * (-b + det) / a;
        else
            x1 = 2.0 * c / (-b - det);
        if (Math.abs(-b + det) < Math.abs(a))
            x2 = 0.5 * (-b - det) / a;
        else
            x2 = 2.0 * c / (-b + det);
        return List.of(x1, x2);
    }

    static double Sqr(double x) {
        return x * x;
    }

    static boolean FuzzyZero(double a, double eps) {
        return Math.abs(a) <= eps;
    }

    static boolean FuzzyEquals(double a, double b, double eps) {
        return Math.abs(a - b) <= eps;
    }

    static class Outvar<T> {
        T v;

        public Outvar(T v) {
            this.v = v;
        }
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces.
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

    // TODO remove
    @Override
    public Path<N4> getBestPath() {
        return _sigma_best;
    }

    @Override
    public SinglePath<N4> getBestSinglePath() {
        return _single_sigma_best;
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

    /**
     * Return a path starting the leaf, ending at the root.
     * 
     * Return a list of links starting from the leaf, ending at the root.
     * 
     * @param node leaf of a tree
     */
    SinglePath<N4> walkParentsSingle(Set<Node<N4>> visited, Node<N4> node) {
        // Collect the states along the path (backwards)
        // List<Matrix<N4, N1>> configs = new LinkedList<>();
        List<SinglePath.Link<N4>> links = new LinkedList<>();

        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance

        while (true) {
            if (visited.contains(node)) {
                System.out.println("found a cycle");
                throw new IllegalArgumentException();
            }
            visited.add(node);

            // configs.add(node.getState());
            LinkInterface<N4> incoming = node.getIncoming();
            if (incoming == null)
                break;
            // walking backwards here, so the "source" is actually the "target" of the link.
            SinglePath.Link<N4> link = new SinglePath.Link<>(
                    incoming.get_target().getState(),
                    incoming.get_source().getState(),
                    incoming.get_linkDist());
            links.add(link);
            node = incoming.get_source();
        }
        // we collected these from leaf to root, so reverse to get root to leaf.

        // new, no more reversing.

        // Collections.reverse(configs);
        // reversing the list of links entails reversing each link too.
        // but note that the caller is going to reverse one of them again,
        // so let's not do that.

        return new SinglePath<>(/* configs, */ links);
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
        // see below
        // this.radius = _gamma * Math.pow(Math.log(stepNo + 1) / (stepNo + 1), 0.25);
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    static boolean same(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        return a.isEqual(b, 0.0001);
    }
}