package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;

import org.team100.lib.graph.Graph;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.NearNode;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDModel;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.planner.RobotModel;
import org.team100.lib.planner.Solver;
import org.team100.lib.space.Path;
import org.team100.lib.space.Sample;
import org.team100.lib.space.SinglePath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * RRT* version 4. this is the BIDIRECTIONAL version.
 * 
 * This is an attempt to make the code below look more like the pseudocode.
 * 
 * There are many similar-but-not-identical variations; I've kind of mashed them
 * together.
 * 
 * x_rand <- Sample() // a random point
 * x_nearest <- Nearest(x_rand) // the nearest node in the tree
 * x_new <- Steer(x_nearest, x_rand) // a feasible point nearby
 * if CollisionFree then
 * | X_near = Near(x_new) // candidate parents
 * | x_min = ChooseParent(X_near, x_nearest, x_new) // lowest cost parent
 * | if x_min != null then
 * | | newNode = InsertNode(x_min, x_new)
 * | | Rewire(X_near, newNode);
 * 
 * References:
 * 
 * http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
 * https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/SamplingBasedMotionPlanning.pdf
 * http://lavalle.pl/planning/book.pdf
 * https://dspace.mit.edu/handle/1721.1/78449
 * https://dspace.mit.edu/handle/1721.1/79884
 * https://natanaso.github.io/ece276b2018/ref/ECE276B_8_SamplingBasedPlanning.pdf
 * https://dspace.mit.edu/bitstream/handle/1721.1/79884/MIT-CSAIL-TR-2013-021.pdf
 * 
 * This version implements bidirectional search, which is consisely mentioned
 * here
 * 
 * https://arxiv.org/pdf/1703.08944.pdf
 */
public class RRTStar4<States extends Num, T extends KDModel<States> & RobotModel<States>> implements Solver<States> {
    private final T _model;
    /** Initially, tree grown from initial, but is swapped repeatedly */
    private KDNode<Node<States>> _T_a;
    /** Initially, tree grown from goal, but is swapped repeatedly */
    private KDNode<Node<States>> _T_b;
    private final Sample<States> _sample;
    private final double _gamma;

    // mutable loop variables to make the loop code cleaner
    private int stepNo;
    private double radius;

    private Path<States> _sigma_best;
    private Map<Node<States>, Node<States>> connections = new HashMap<>();

    public RRTStar4(T model, Sample<States> sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _T_a = new KDNode<>(new Node<>(model.initial()));
        _T_b = new KDNode<>(new Node<>(model.goal()));
        _sample = sample;
        _gamma = gamma;
    }

    /**
     * Note this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it doesn't use Extend, so it doesn't try to connect unless
     * a new node is actually inserted.
     * 
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        int edges = 0;
        Matrix<States, N1> x_rand = SampleFree();
        KDNearNode<Node<States>> x_nearest = Nearest(x_rand, _T_a);
        Matrix<States, N1> x_new = Steer(x_nearest, x_rand);
        if (CollisionFree(x_nearest._nearest.getState(), x_new)) {
            List<NearNode<States>> X_near = Near(x_new, _T_a);
            if (X_near.isEmpty())
                X_near.add(new NearNode<>(x_nearest._nearest, x_nearest._dist));
            Node<States> x_min = ChooseParent(X_near, x_new);
            if (x_min != null) {
                Node<States> newNode = InsertNode(x_min, x_new, _T_a);
                Rewire(X_near, newNode);
                edges += 1;
                KDNearNode<Node<States>> x_conn = Nearest(x_new, _T_b);
                Path<States> sigma_new = Connect(newNode, x_conn, _T_b);
                if (sigma_new != null) {
                    edges += 1;
                    if (_sigma_best == null) {
                        _sigma_best = sigma_new;
                    } else {
                        if (sigma_new.getDistance() < _sigma_best.getDistance()) {
                            _sigma_best = sigma_new;
                        }
                    }
                }
            }
        }
        SwapTrees();
        return edges;
    }

    void SwapTrees() {
        KDNode<Node<States>> tmp = _T_a;
        _T_a = _T_b;
        _T_b = tmp;
    }

    /**
     * this isn't quite the same as https://arxiv.org/pdf/1703.08944.pdf
     * because it skips the "extend," because i think it's wrong; it just
     * picks nodes in the other tree that are actually near the new node.
     * 
     * @param x_1 newly inserted node
     * @param x_2 near node in the other tree
     */
    Path<States> Connect(Node<States> x_1, KDNearNode<Node<States>> x_2, KDNode<Node<States>> rootNode) {
        if (CollisionFree(x_2._nearest.getState(), x_1.getState())) {
            List<NearNode<States>> X_near = Near(x_1.getState(), rootNode);
            if (X_near.isEmpty())
                X_near.add(new NearNode<>(x_2._nearest, x_2._dist));
            Node<States> x_min = ChooseParent(X_near, x_1.getState());
            if (x_min != null) {
                Node<States> newNode = InsertNode(x_min, x_1.getState(), rootNode);
                connections.put(x_1, newNode);
                Rewire(X_near, newNode);
                return GeneratePath(x_1, newNode);
            }
        }
        return null;
    }

    /**
     * the parameters describe a link between initial and goal trees, the same
     * state in both cases.
     */
    Path<States> GeneratePath(Node<States> x_1, Node<States> x_2) {
        if (!x_1.getState().isEqual(x_2.getState(), 0.001))
            throw new IllegalArgumentException(
                    "x1 " + x_1.getState().toString() + " != x2 " + x_2.getState().toString());
        Path<States> p_1 = walkParents(x_1);
        Path<States> p_2 = walkParents(x_2);
        List<Matrix<States, N1>> states_2 = p_2.getStatesA();
        Collections.reverse(states_2);
        return new Path<>(p_1.getDistance() + p_2.getDistance(), p_1.getStatesA(), states_2);
    }

    boolean CollisionFree(Matrix<States, N1> from, Matrix<States, N1> to) {
        return _model.link(from, to);
    }

    /** Return a state not within an obstacle. */
    Matrix<States, N1> SampleFree() {
        while (true) {
            Matrix<States, N1> newConfig = _sample.get();
            if (_model.clear(newConfig))
                return newConfig;
        }
    }

    /**
     * Return the nearest node in the tree, using the KDTree metric, which
     * could be very wrong but is probably useful.
     * 
     * @param x_rand   the random sample
     * @param rootNode the tree to look through
     */
    KDNearNode<Node<States>> Nearest(Matrix<States, N1> x_rand, KDNode<Node<States>> rootNode) {
        return KDTree.nearest(_model, rootNode, x_rand);
    }

    /**
     * Return a state that tries to go from x_nearest to x_rand using a feasible
     * trajectory. For simple systems "feasible" just means "closer."
     * 
     * Returns x_rand unmodified if it's within radius.
     * 
     * @param x_nearest containts distance to x_rand
     */
    Matrix<States, N1> Steer(KDNearNode<Node<States>> x_nearest, Matrix<States, N1> x_rand) {
        if (x_nearest._dist < radius) {
            return x_rand;
        }
        _model.setStepNo(stepNo);
        _model.setRadius(radius);
        return _model.steer(x_nearest, x_rand);
    }

    /**
     * Return a list of nearby nodes, using the KDTree metric, which may not
     * actually contain the nearest nodes in non-Euclidean spaces. Returns the
     * single
     * nearest node if there are no other near nodes.
     */
    List<NearNode<States>> Near(Matrix<States, N1> x_new, KDNode<Node<States>> rootNode) {
        List<NearNode<States>> nearNodes = new ArrayList<>();
        KDTree.near(_model, rootNode, x_new, radius, (node, dist) -> nearNodes.add(new NearNode<>(node, dist)));
        return nearNodes;
    }

    /**
     * Returns a member of X_near resulting in the lowest-cost path to x_new.
     * Removes infeasible nodes from X_near so we don't look at them again later.
     */
    Node<States> ChooseParent(List<NearNode<States>> X_near, Matrix<States, N1> x_new) {
        Collections.sort(X_near);
        Iterator<NearNode<States>> ni = X_near.iterator();
        while (ni.hasNext()) {
            NearNode<States> nearNode = ni.next();
            ni.remove();
            if (CollisionFree(nearNode.node.getState(), x_new)) {
                return nearNode.node;
            }
        }
        return null;
    }

    /** Add the node x_new to the tree, with an edge from x_min. */
    Node<States> InsertNode(Node<States> x_min, Matrix<States, N1> x_new, KDNode<Node<States>> rootNode) {
        Node<States> newNode = new Node<>(x_new);
        Graph.newLink(_model, x_min, newNode);
        KDTree.insert(_model, rootNode, newNode);
        return newNode;
    }

    /**
     * look through the nodes in X_near to see if any should be new children of
     * newNode.
     */
    void Rewire(List<NearNode<States>> X_near, Node<States> newNode) {
        ListIterator<NearNode<States>> li = X_near.listIterator(X_near.size());
        while (li.hasPrevious()) {
            NearNode<States> jn = li.previous();
            if (jn.node.getIncoming() != null) {
                Graph.rewire(_model, newNode, jn.node, jn.linkDist);
            }
        }
    }

    @Override
    public List<Node<States>> getNodesA() {
        List<Node<States>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_a));
        return allNodes;
    }

    @Override
    public List<Node<States>> getNodesB() {
        List<Node<States>> allNodes = new ArrayList<>();
        allNodes.addAll(KDTree.values(_T_b));
        return allNodes;
    }

    /**
     * the path distance may have been changed by rewiring. does this actually
     * matter? experiment says no.
     */
    public Path<States> getFullBestPath() {
        Path<States> bestPath = null;
        for (Map.Entry<Node<States>, Node<States>> entry : connections.entrySet()) {
            Path<States> aPath = GeneratePath(entry.getKey(), entry.getValue());
            if (bestPath == null) {
                bestPath = aPath;
            } else {
                if (aPath.getDistance() < bestPath.getDistance())
                    bestPath = aPath;
            }
        }
        return bestPath;
    }

    @Override
    public Path<States> getBestPath() {
        return _sigma_best;
    }

    /**
     * Starting from leaf node, walk the parent links to accumulate
     * the full path, and reverse it, to return a path from root to node.
     * 
     * @param node leaf node
     */
    Path<States> walkParents(Node<States> node) {
        // Collect the states along the path (backwards)
        List<Matrix<States, N1>> configs = new LinkedList<>();
        // Since we're visiting all the nodes it's very cheap to verify the total
        // distance
        double totalDistance = 0;
        while (true) {
            configs.add(node.getState());
            LinkInterface<States> incoming = node.getIncoming();
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
        this.radius = _gamma * Math.pow(
                Math.log(stepNo + 1.0) / (stepNo + 1.0),
                1.0 / _T_a.getValue().getState().getNumRows());
    }

    @Override
    public SinglePath<States> getBestSinglePath() {
        throw new UnsupportedOperationException("Unimplemented method 'getBestSinglePath'");
    }
}