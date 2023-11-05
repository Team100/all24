package org.team100.lib.rrt;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

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

public class RRTStar<States extends Num, T extends KDModel<States> & RobotModel<States>> implements Solver<States> {
    private final T _model;
    private final KDNode<Node<States>> _rootNode;
    private final Sample<States> _sample;
    private final double _gamma;
    private LinkInterface<States> _bestPath;

    // mutable loop variables to make the loop code cleaner
    int stepNo = 0;
    Matrix<States, N1> x_rand;

    public RRTStar(T model, Sample<States> sample, double gamma) {
        if (gamma < 1.0) {
            throw new IllegalArgumentException("invalid gamma, must be >= 1.0");
        }
        _model = model;
        _rootNode = new KDNode<>(new Node<>(model.initial()));
        _sample = sample;
        _gamma = gamma;
        _bestPath = null;
    }

    Matrix<States, N1> SampleFree() {
        Matrix<States, N1> newConfig = _sample.get();
        if (!_model.clear(newConfig))
            return null;
        return newConfig;
    }

    @Override
    public void setStepNo(int stepNo) {
        if (stepNo < 1)
            throw new IllegalArgumentException();
        this.stepNo = stepNo;
    }

    /**
     * @return true if a new sample was added.
     */
    @Override
    public int step() {
        x_rand = SampleFree();
        if (x_rand == null)
            return 0;

        double radius = _gamma * Math.pow(
                Math.log(stepNo + 1.0) / (stepNo + 1.0),
                1.0 /  _rootNode.getValue().getState().getNumRows());

        List<NearNode<States>> nearNodes = new ArrayList<>();
        KDTree.near(_model, _rootNode, x_rand, radius, (node, dist) -> {
            if (node.getIncoming() != null)
                nearNodes.add(new NearNode<>(node, dist));
        });

        if (nearNodes.isEmpty()) {

            KDNearNode<Node<States>> nearResult = KDTree.nearest(_model, _rootNode, x_rand);
            Node<States> nearest = nearResult._nearest;
            double distToNearest = nearResult._dist;

            if (distToNearest > radius) {
                _model.setStepNo(stepNo);
                _model.setRadius(radius);
                x_rand = _model.steer(nearResult, x_rand);
            }

            if (!_model.clear(x_rand)) {
                return 0;
            }

            if (!_model.link(nearest.getState(), x_rand)) {
                return 0;
            }

            // the new node has the new sampled config, the distance(cost) to the
            // nearest other node we found above, and the "parent" is the "link"
            // from that nearest node.
            Node<States> newTarget = new Node<>(x_rand);
            // recalculate dist just to be safe.
            LinkInterface<States> newLink = Graph.newLink(_model, nearest, newTarget);

            _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);

            KDTree.insert(_model, _rootNode, newTarget);
            return 1;
        }

        // Sort the array by total distance (including the distance to the new node).
        // We take the best (shortest) feasible node.
        Collections.sort(nearNodes);

        Iterator<NearNode<States>> ni = nearNodes.iterator();
        while (ni.hasNext()) {
            NearNode<States> nearNode = ni.next();
            ni.remove();

            if (!_model.link(nearNode.node.getState(), x_rand)) {
                continue;
            }

            // Found a linkable configuration.
            Node<States> newNode = new Node<>(x_rand);
            LinkInterface<States> newLink = Graph.newLink(nearNode.node, newNode, nearNode.linkDist);
            _bestPath = Graph.chooseBestPath(_model, _bestPath, newLink);
            KDTree.insert(_model, _rootNode, newNode);

            // check the remaining nearby nodes to see if they would be better
            // as children of the new node
            ListIterator<NearNode<States>> li = nearNodes.listIterator(nearNodes.size());
            while (li.hasPrevious()) {
                NearNode<States> jn = li.previous();
                if (Graph.rewire(_model, newNode, jn.node, jn.linkDist)) {
                    _bestPath = Graph.chooseBestPath(_model, _bestPath, newNode.getIncoming());
                }
            }
            return 1;
        }
        // no feasible link possible.
        return 0;
    }

    @Override
    public List<Node<States>> getNodesA() {
        return KDTree.values(_rootNode);
    }

    @Override
    public List<Node<States>> getNodesB() {
        return new ArrayList<>();
    }

    @Override
    public Path<States> getBestPath() {
        LinkInterface<States> link = _bestPath;
        if (link == null) {
            return null;
        }
        Node<States> node = link.get_target();

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
    public SinglePath<States> getBestSinglePath() {
        throw new UnsupportedOperationException("Unimplemented method 'getBestSinglePath'");
    }
}