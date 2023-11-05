package org.team100.lib.graph;

import java.util.Iterator;

import org.team100.lib.index.KDModel;
import org.team100.lib.planner.RobotModel;

import edu.wpi.first.math.Num;

public class Graph {
    /** use the caching link type? */
    public static boolean linkTypeCaching = true;

    /**
     * Create a link from source to target, using the model distance from the source
     * to target.
     */
    public static <States extends Num> LinkInterface<States> newLink(KDModel<States> model, Node<States> source,
            Node<States> target) {
        double dist = model.dist(source.getState(), target.getState());
        // System.out.printf("newLink from [%5.3f %5.3f] to [%5.3f %5.3f] d %5.3f\n",
        // source.getState()[0], source.getState()[1], target.getState()[0],
        // target.getState()[1], dist);
        return newLink(source, target, dist);
    }

    /**
     * Create a link from source to target, add it to the source outgoing set, and
     * set it as the target incoming link. Also remove the target's previous
     * incoming link from its source outgoing set, if needed.
     * 
     * @return the newly created link, never null.
     */
    public static <States extends Num> LinkInterface<States> newLink(Node<States> source, Node<States> target,
            double dist) {
        if (source == null)
            throw new IllegalArgumentException("source may not be null");
        if (target == null)
            throw new IllegalArgumentException("target may not be null");
        if (dist < 0)
            throw new IllegalArgumentException("dist may not be negative");

        LinkInterface<States> oldLink = target.getIncoming();
        if (oldLink != null)
            oldLink.get_source().removeOutgoing(oldLink);

        LinkInterface<States> link;
        if (linkTypeCaching) {
            link = new PathDistanceCachingLink<>(source, target, dist);
        } else {
            link = new LocalLink<>(source, target, dist);
        }
        target.setIncoming(link);
        source.addOutgoing(link);
        return link;
    }

    /**
     * Walks the incoming path to calculate the total path distance to the specified
     * node.
     */
    public static <States extends Num> double getPathDist(Node<States> node) {
        double pathDist = 0;
        while (true) {
            LinkInterface<States> incoming = node.getIncoming();
            if (incoming == null)
                return pathDist;
            pathDist += incoming.get_linkDist();
            node = incoming.get_source();
        }
    }

    /** Walks the outgoing subtree and updates the path lengths of each link. */
    public static <States extends Num> void updatePathLengths(LinkInterface<States> link) {
        Node<States> node = link.get_target();
        Iterator<LinkInterface<States>> iter = node.getOutgoing();
        while (iter.hasNext()) {
            LinkInterface<States> child = iter.next();
            child.set_PathDist(child.get_linkDist() + link.get_pathDist());
            updatePathLengths(child);
        }
    }

    /**
     * @return best path (link or bestpath, whichever is shorter)
     */
    public static <States extends Num> LinkInterface<States> chooseBestPath(
            RobotModel<States> model,
            final LinkInterface<States> oldLink,
            final LinkInterface<States> newLink) {
        // System.out.println("try new best path");
        if (newLink == null)
            throw new IllegalArgumentException();

        if (!model.goal(newLink.get_target().getState()))
            return oldLink;

        if (oldLink == null)
            return newLink;

        if (newLink.get_pathDist() < oldLink.get_pathDist()) {
            // System.out.printf("new best path %5.3f\n", newLink.get_pathDist());
            return newLink;
        }

        return oldLink;
    }

    /**
     * Rewires the target node to source.
     * 
     * @return true if it actually changed anything
     */
    public static <States extends Num> boolean rewire(RobotModel<States> _robotModel, Node<States> source, Node<States> target, double linkDist) {
        if (target.getIncoming() == null)
            throw new IllegalArgumentException("cannot rewire the root");

        if (target.getIncoming().get_source() == source)
            throw new IllegalArgumentException("vacuous rewiring");

        double newPathDist = source.getPathDist() + linkDist;

        // if the new path is not better, then return the old path
        if (newPathDist >= target.getIncoming().get_pathDist()) {
            return false;
        }

        // if the new link is not feasible, return the old path
        if (!_robotModel.link(target.getState(), source.getState())) {
            return false;
        }

        // actually make and set the new link
        LinkInterface<States> oldLink = target.getIncoming();
        oldLink.get_source().removeOutgoing(oldLink);
        LinkInterface<States> newLink = newLink(source, target, linkDist);

        // Update all the child path lengths for consistency.
        // but only for the link types that need it.
        if (linkTypeCaching)
            updatePathLengths(newLink);

        return true;
    }
}
