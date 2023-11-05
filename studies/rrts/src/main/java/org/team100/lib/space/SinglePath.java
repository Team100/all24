package org.team100.lib.space;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * paths from both trees joined into a single list.
 *
 * TODO: add control states here
 */
public class SinglePath<States extends Num> {
    public static class Link<States extends Num> {
        public final Matrix<States, N1> x_i;
        public final Matrix<States, N1> x_g;
        public final double cost;

        public Link(Matrix<States, N1> x_i, Matrix<States, N1> x_g, double cost) {
            this.x_i = x_i;
            this.x_g = x_g;
            this.cost = cost;
        }

        @Override
        public String toString() {
            return "Link [x_i=" + x_i + ", x_g=" + x_g + ", cost=" + cost + "]";
        }
    }

    private final List<Link<States>> links;

    public SinglePath(List<Link<States>> links) {
        this.links = links;

        // invariant: x_g of one link is x_i of the next.
        for (int i = 0; i < links.size() - 1; ++i) {
            Link<States> link1 = links.get(i);
            Link<States> link2 = links.get(i + 1);
            if (!link1.x_g.isEqual(link2.x_i, 0.001)) {
                System.out.printf("%d %s %s\n", i, link1.x_g, link2.x_i);
                throw new IllegalArgumentException();
            }
        }
    }

    /** Sum of link cost via full scan. */
    public double getDistance() {
        double result = 0;
        for (Link<States> l:links) {
            result += l.cost;
        }
        return result;
    }

    /** return a copy of the link list */
    public List<Link<States>> getLinks() {
        List<Link<States>> allLinks = new ArrayList<>();
        allLinks.addAll(links);
        return allLinks;
    }

    public Link<States> getFirstLink() {
        return links.get(0);
    }

    public Link<States> getLastLink() {
        return links.get(links.size() - 1);
    }

    @Override
    public String toString() {
        return "SinglePath [links=" + links + "]";
    }
}
