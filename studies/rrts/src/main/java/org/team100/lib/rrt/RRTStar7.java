package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;
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
import org.team100.lib.planner.Solver;
import org.team100.lib.random.MersenneTwister;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.space.SinglePath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;

/**
 * RRT* version 7
 * 
 * Full-state 4d field, following the bang-bang rrt paper.
 * 
 * Papers referenced below:
 * 
 * [1] LaSalle et al, Bang-Bang RRT, 2023. https://arxiv.org/pdf/2210.01744.pdf
 * [2] Hauser et al, Optimal shortcuts, 2010,
 * https://motion.cs.illinois.edu/papers/icra10-smoothing.pdf
 */
public class RRTStar7<T extends Arena<N4>> implements Solver<N4> {
    public static boolean DEBUG = false;
    private static final double MAX_U = 2.5;
    private static final boolean BIDIRECTIONAL = true;

    private final T _model;
    private final Sample<N4> _sample;
    private final Random random = new MersenneTwister(new Random().nextInt());

    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node<N4>> _T_b;

    // mutable loop variables to make the loop code cleaner
    private double radius;

    private SinglePath<N4> _single_sigma_best;

    static boolean PARTIAL = true;

    public RRTStar7(T model, Sample<N4> sample, KDNode<Node<N4>> T_a, KDNode<Node<N4>> T_b) {
        _model = model;
        _sample = sample;
        _T_a = T_a;
        _T_b = T_b;
    }

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
        if (x_nearestA == null) {
            if (DEBUG)
                System.out.println("nothing near");
            return 0;
        }

        // includes states and controls
        // note the resulting trajectory is reversed if time is reversed.
        // for first half of bidirectional, partial is ok.
        Trajectory phiA = null;
        if (timeForward) {
            // we're looking in T_a so the new node is later
            phiA = BangBangSteer(_model::clear, x_nearestA._nearest.getState(), x_rand, true, true);
        } else {
            // we're looking in T_b so the new node is earlier
            phiA = BangBangSteer(_model::clear, x_rand, x_nearestA._nearest.getState(), false, true);
        }
        if (phiA == null)
            return 0;

        if (DEBUG)
            System.out.println(phiA);

        // now we have a clear trajectory from nearest to rand.

        double tMaxA = Math.max(phiA.x.s1.t + phiA.x.s2.t, phiA.y.s1.t + phiA.y.s2.t);
        final Matrix<N4, N1> x_iA = new Matrix<>(Nat.N4(), Nat.N1(),
                new double[] { phiA.x.i, phiA.x.idot, phiA.y.i, phiA.y.idot });
        final Matrix<N4, N1> x_gA = new Matrix<>(Nat.N4(), Nat.N1(),
                new double[] { phiA.x.g, phiA.x.gdot, phiA.y.g, phiA.y.gdot });

        // in partial mode, the resulting state might not be at the probe point.
        if (!PARTIAL) {
            if (timeForward) {
                // the probe point should be the goal state
                if (!x_rand.isEqual(x_gA, 0.001))
                    throw new IllegalArgumentException();
            } else {
                // the probe point should be the initial state
                if (!x_rand.isEqual(x_iA, 0.001))
                    throw new IllegalArgumentException();
            }
        }

        Node<N4> freeEndA = null;

        {
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
                    Node<N4> target = new Node<>(x_gA);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tMaxA - tSoFar);
                    InsertNode(randLink, _T_a);
                }
                if (!PARTIAL) {
                    // at this point the "leaf" of A should be "goal" which is the probe point
                    if (!x_rand.isEqual(freeEndA.getState(), 0.001))
                        throw new IllegalArgumentException();
                    if (!x_gA.isEqual(freeEndA.getState(), 0.001))
                        throw new IllegalArgumentException();
                }
            } else {
                // time is reversed, so this is T_b, so walk the trajectory backwards
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
                    if (DEBUG)
                        System.out.printf("A2 last state %s\n", x_iA);
                    Node<N4> target = new Node<>(x_iA);
                    freeEndA = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tSoFar);
                    InsertNode(randLink, _T_a);
                }
                if (!PARTIAL) {
                    // in the reversed case, the "leaf" is the "initial" which is the probe point
                    if (!x_rand.isEqual(freeEndA.getState(), 0.001))
                        throw new IllegalArgumentException();
                    if (!x_iA.isEqual(freeEndA.getState(), 0.001))
                        throw new IllegalArgumentException();
                }
            }
        }

        // at this point we should use freeEndA as the join point, not x_iA or x_gA.

        // SwapTrees();

        edges += 1;

        if (BIDIRECTIONAL) {

            // now check for feasible paths to some node in the other tree.
            // note that the continuity requirement is not to match the state, it's to match
            // the time-reversed state, since the two trees have opposite time polarity.

            // note we use the *actual* end state of the previous trajectory, not the
            // original x_rand.
            KDNearNode<Node<N4>> x_nearestB = BangBangNearest(freeEndA.getState(), _T_b, !timeForward);
            if (x_nearestB == null) {
                SwapTrees();
                return 1;
            }

                            // for the second half of bidirectional don't allow partial solutions
            Trajectory phiB = null;
            if (timeForward) {
                // we're looking in T_b which is later than T_a so the join is the initial
                // state.
                phiB = BangBangSteer(_model::clear, freeEndA.getState(), x_nearestB._nearest.getState(), timeForward, false);
            } else {
                // we're looking in T_a which is earlier than T_b so the join is the goal state
                phiB = BangBangSteer(_model::clear, x_nearestB._nearest.getState(), freeEndA.getState(), timeForward, false);
            }
            if (phiB == null) {
                SwapTrees();
                return 1;
            }
            double tMaxB = Math.max(phiB.x.s1.t + phiB.x.s2.t, phiB.y.s1.t + phiB.y.s2.t);
            final Matrix<N4, N1> x_i = new Matrix<>(Nat.N4(), Nat.N1(),
                    new double[] { phiB.x.i, phiB.x.idot, phiB.y.i, phiB.y.idot });
            final Matrix<N4, N1> x_g = new Matrix<>(Nat.N4(), Nat.N1(),
                    new double[] { phiB.x.g, phiB.x.gdot, phiB.y.g, phiB.y.gdot });

            if (!PARTIAL) {
                if (timeForward) {
                    // if we're looking in the "other" tree in time-forward mode the probe
                    // point is the initial state
                    if (!x_rand.isEqual(x_i, 0.001)) {
                        System.out.printf("%s %s %s\n", x_rand, x_i, x_g);
                        throw new IllegalArgumentException();
                    }
                } else {
                    // if we're looking in the "other" tree in the time-reversed mode then we're
                    // looking at T_a, so the probe is the goal point.
                    if (!x_rand.isEqual(x_g, 0.001)) {
                        System.out.printf("%s %s %s\n", x_rand, x_i, x_g);
                        throw new IllegalArgumentException();
                    }
                }
            }

            // the returned endpoint may not actually be the same as the desired endpoint,
            // which now means we should just not make the link, i think.
            if (timeForward) {
                // the B tree really is the backwards one, so the leaf is the "initial" node of
                // the trajectory
                if (!x_i.isEqual(freeEndA.getState(), 0.001)) {
                    System.out.printf("%s %s %s\n", x_i, x_g, freeEndA.getState());
                    // throw new IllegalArgumentException();
                    SwapTrees();
                    return 1;
                }
            } else {
                // the B tree is the forwards one, so the leaf is the "goal" node of the
                // trajectory
                if (!x_g.isEqual(freeEndA.getState(), 0.001)) {
                    System.out.printf("%s %s %s\n", x_i, x_g, freeEndA.getState());
                    // throw new IllegalArgumentException();
                    SwapTrees();
                    return 1;
                }
            }

            Node<N4> freeEndB = null;
            // make lots of little segments
            double tStep = 0.1;
            Node<N4> source = x_nearestB._nearest;
            if (DEBUG)
                System.out.printf("phiB %s\n", phiB);
            if (timeForward) {
                double tSoFar = 0;
                for (double tSec = tMaxB; tSec >= 0; tSec -= tStep) {
                    tSoFar = tSec;
                    // running the trajectory backwards to build tree B
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
                    // the t=0 state is the initial state x_i
                    // add one more segment to actually reach the goal
                    if (DEBUG)
                        System.out.printf("A4 last state %s\n", x_i);
                    Node<N4> target = new Node<>(x_i);
                    freeEndB = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tSoFar);
                    InsertNode(randLink, _T_b);
                }
                if (!PARTIAL) {
                    // in the forward case for the "other" tree the "leaf" is the "initial" which is
                    // the probe point
                    if (!x_rand.isEqual(freeEndB.getState(), 0.001))
                        throw new IllegalArgumentException();
                    if (!x_i.isEqual(freeEndB.getState(), 0.001))
                        throw new IllegalArgumentException();
                }
            } else {
                double tSoFar = 0;
                for (double tSec = tStep; tSec <= tMaxB; tSec += tStep) {
                    tSoFar = tSec;
                    Matrix<N4, N1> state = SampleTrajectory(phiB, tSec);
                    if (DEBUG)
                        System.out.printf("A3 stepstate %s\n", state);
                    Node<N4> target = new Node<>(state);
                    freeEndB = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tMaxB - tSoFar);
                    InsertNode(randLink, _T_b);
                    source = target;
                }
                if (tSoFar < tMaxB) {
                    // add one more segment to actually reach the goal
                    if (DEBUG)
                        System.out.printf("A3 last state %s\n", x_g);

                    Node<N4> target = new Node<>(x_g);
                    // this seems correct: time reversed means T_b is actually T_a so the trajectory
                    // goal is the join point
                    freeEndB = target;
                    LocalLink<N4> randLink = new LocalLink<>(source, target, tStep);
                    InsertNode(randLink, _T_b);
                }
                if (!PARTIAL) {
                    // in the reverse case for the "other" tree the "leaf" is the "goal" which is
                    // the probe point
                    if (!x_rand.isEqual(freeEndB.getState(), 0.001))
                        throw new IllegalArgumentException();
                    if (!x_g.isEqual(freeEndB.getState(), 0.001))
                        throw new IllegalArgumentException();
                }
            }

            // if we got here then we are finished constructing a path, and should stop
            // and start optimizing it.

            // for now make a "path"

            if (freeEndB != null) {
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

            SwapTrees();
        }
        return edges;
    }

    public void SwapTrees() {
        KDNode<Node<N4>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    public void Optimize(double frac1, double frac2) {
        final List<SinglePath.Link<N4>> links = _single_sigma_best.getLinks();

        final int nodect = links.size();
        final int node1 = (int) (nodect * frac1);
        final int node2 = (int) (nodect * frac2);

        if (DEBUG)
            System.out.printf("%d %d\n", node1, node2);

        // now node1 is first
        // actually we want the sub-list
        List<SinglePath.Link<N4>> sublist = links.subList(node1, node2 + 1);
        // these could be the same, just replace a single link.
        final Matrix<N4, N1> state1 = sublist.get(0).x_i;
        final Matrix<N4, N1> state2 = sublist.get(sublist.size() - 1).x_g;
        double cost = 0;
        for (SinglePath.Link<N4> link : sublist) {
            cost += link.cost;
        }

        // try to get there

        // always returns the given intial state.
        // for optimization never allow partial solutions.
        Trajectory phiA = BangBangSteer(_model::clear, state1, state2, true, false);
        if (phiA == null)
            return;

        

        double tMaxA = Math.max(phiA.x.s1.t + phiA.x.s2.t, phiA.y.s1.t + phiA.y.s2.t);

        if (tMaxA >= cost)
            return;

        // we can do better
        final List<SinglePath.Link<N4>> replacement = new ArrayList<>();

        double tStep = 0.1;

        // use the returned trajectory endpoints
        // final Matrix<N4,N1> state_i = new Matrix<>(Nat.N4(), Nat.N1(), new double[] {
        // phiA.x.i, phiA.x.idot, phiA.y.i, phiA.y.idot });
        final Matrix<N4, N1> state_g = new Matrix<>(Nat.N4(), Nat.N1(),
                new double[] { phiA.x.g, phiA.x.gdot, phiA.y.g, phiA.y.gdot });

        if (!state_g.isEqual(state2, 0.001)) {
            // didn't make it, so give up
            return;
        }

        double tSoFar = 0;
        // Matrix<N4,N1> state_interior = state_i;
        Matrix<N4, N1> state_interior = state1;
        for (double tSec = tStep; tSec <= tMaxA; tSec += tStep) {
            tSoFar = tSec;
            final Matrix<N4, N1> state = SampleTrajectory(phiA, tSec);
            SinglePath.Link<N4> randLink = new SinglePath.Link<>(state_interior, state, tStep);
            replacement.add(randLink);
            state_interior = state;
        }
        if (tSoFar < tMaxA) {
            // add one more segment to actually reach xrand
            SinglePath.Link<N4> randLink = new SinglePath.Link<>(state_interior, state_g, tMaxA - tSoFar);
            replacement.add(randLink);
        }

        // this removes the sublist items from the links list
        sublist.clear();
        // insert the replacement links at the node1 position
        links.addAll(node1, replacement);
        _single_sigma_best = new SinglePath<>(links);

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
     * @return the nearest node, which will be earlier than xNew if time is forward,
     *         and later if time is reversed.
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
                // time forward means xNew is in the future
                tOptimal = tOptimal(node.node.getState(), xNew, MAX_U);
            } else {
                // time backward means xNew is in the past
                tOptimal = tOptimal(xNew, node.node.getState(), MAX_U);
            }
            if (tOptimal < tMin) {
                tMin = tOptimal;
                bestNode = node.node;
            }
        }
        if (tMin == Double.MAX_VALUE) {
            if (DEBUG)
                System.out.println("no optimal time");
            return null;
        }

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
            double umax) {

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
     * until it hits something. This results in a "doomed" trajectory that will
     * never be picked up from the end, but the intermediate states are useful.
     * 
     * So instead i'll just check for collisions, return the trajectory if there
     * aren't any, and otherwise return null.
     *
     * if time is reversed the caller should swap the arguments so that x_i is
     * temporally earlier.
     * 
     * @param free        predicate indicating collision-free
     * @param x_i         initial state, always earlier than the goal.
     * @param x_g         goal state
     * @param timeForward find partial states in the correct direction using this
     *                    parameter
     * @param partialOK allow partial solutions
     * @return a feasible trajectory that starts at x_i and ends somewhere on the
     *         path to x_g, or null if no trajectory is
     *         feasible at all.
     */
    static Trajectory BangBangSteer(
            Predicate<Matrix<N4, N1>> free,
            Matrix<N4, N1> x_i,
            Matrix<N4, N1> x_g,
            boolean timeForward,
            boolean partialOK) {
        if (DEBUG)
            System.out.printf("attempt %s %s\n", x_i, x_g);
        Trajectory trajectory = optimalTrajectory(x_i, x_g, MAX_U);
        if (trajectory == null)
            return null;
        if (DEBUG)
            System.out.printf("produced %s\n", trajectory);
        double tMax = Math.max(trajectory.x.s1.t + trajectory.x.s2.t, trajectory.y.s1.t + trajectory.y.s2.t);
        double tStep = 0.1;

        if (timeForward) {
            // walking forward in time is fine
            Matrix<N4, N1> goodState = null;
            for (double tSec = 0; tSec < tMax; tSec += tStep) {
                Matrix<N4, N1> state = SampleTrajectory(trajectory, tSec);
                if (DEBUG)
                    System.out.printf("test state %s\n", state);
                if (!free.test(state)) {
                    if (DEBUG)
                        System.out.printf("failed state %s %s\n", x_i, state);
                    if (goodState == null)
                        return null; // nothing feasible at all.
                    // we want to return a partial trajectory here. maybe the optimal one will just
                    // work.
                    // hm, no, it seems to barely miss the goal and so it goes all the way around
                    //
                    if (!partialOK)
                        return null;
                    // try the same trajectory with a bit more U so we won't miss it.
                    Trajectory partial = optimalTrajectory(x_i, goodState, MAX_U + 0.01);
                    if (partial == null)
                        return null;
                    double tMax2 = Math.max(partial.x.s1.t + partial.x.s2.t, partial.y.s1.t + partial.y.s2.t);

                    // verify it one more time
                    for (double tSec2 = 0; tSec2 < tMax2; tSec2 += tStep) {
                        state = SampleTrajectory(partial, tSec2);
                        if (!free.test(state))
                            return null;
                    }

                    if (DEBUG)
                        System.out.printf("return forward partial %s %s %s\n", x_i, goodState, partial);
                    return partial;
                }
                goodState = state;
            }
        } else {
            // find partial solutions by walking backwards in time
            Matrix<N4, N1> goodState = null;
            for (double tSec = tMax; tSec >= 0; tSec -= tStep) {
                Matrix<N4, N1> state = SampleTrajectory(trajectory, tSec);
                if (!free.test(state)) {
                    if (goodState == null)
                        return null; // nothing feasible at all.
                    // we want to return a partial trajectory here. maybe the optimal one will just
                    // work.
                    if (!partialOK)
                        return null;

                    Trajectory partial = optimalTrajectory(goodState, x_g, MAX_U + 0.01);

                    if (partial == null)
                        return null;
                    double tMax2 = Math.max(partial.x.s1.t + partial.x.s2.t, partial.y.s1.t + partial.y.s2.t);

                    // verify it one more time
                    for (double tSec2 = tMax2; tSec2 >= 0; tSec2 -= tStep) {
                        state = SampleTrajectory(partial, tSec2);
                        if (!free.test(state))
                            return null;
                    }
                    if (DEBUG) System.out.printf("return reverse partial %s %s\n", goodState, x_g);
                    return partial;

                }
                goodState = state;
            }
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
    }

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
        if (x_i.isEqual(x_g, 0.001)) {
            return null;
        }
        double tOptimal = tOptimal(x_i, x_g, umax);
        Trajectory result = new Trajectory();
        result.x = slowU(x_i.get(0, 0), x_i.get(1, 0), x_g.get(0, 0), x_g.get(1, 0), tOptimal);
        result.y = slowU(x_i.get(2, 0), x_i.get(3, 0), x_g.get(2, 0), x_g.get(3, 0), tOptimal);
        if (result.x == null || result.y == null)
            return null;
        return result;
    }

    /**
     * This is the method of Hauser et al [1] (see section D) to solve the two-point
     * boundary problem for a single double integrator, minimizing the control
     * output (i.e. current or acceleration). another attempt to find a for fixed T
     * so the paper says solve the quadratic
     * T^2a^2 + sigma(2T(idot+gdot) + 4(i-g))a - (gdot-idot)^2 = 0.
     * 
     * Note that the ParabolicRamp code works differently from this
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
        List<Double> plus = org.team100.lib.math.Util.quadratic(a, b, c);
        // I-G+
        List<Double> minus = org.team100.lib.math.Util.quadratic(a, -b, c);
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
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces.
     */
    ArrayList<NearNode<N4>> Near(Matrix<N4, N1> x_new, KDNode<Node<N4>> rootNode) {
        ArrayList<NearNode<N4>> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> nearNodes.add(new NearNode<>(node, dist)));
        return nearNodes;
    }

    /** Add the node link.target to the tree, with an edge from source to target. */
    Node<N4> InsertNode(LocalLink<N4> link, KDNode<Node<N4>> rootNode) {
        Graph.newLink(link.get_source(), link.get_target(), link.get_linkDist());
        KDTree.insert(_model, rootNode, link.get_target());
        return link.get_target();
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
        return null;
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
        List<SinglePath.Link<N4>> links = new LinkedList<>();

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

        return new SinglePath<>(links);
    }

    @Override
    public void setStepNo(int stepNo) {
        // stepno does nothing
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    static boolean same(Matrix<N4, N1> a, Matrix<N4, N1> b) {
        return a.isEqual(b, 0.0001);
    }
}