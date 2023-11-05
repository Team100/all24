package org.team100.lib.graph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.planner.RobotModel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;

public class OperationsTest {

    RobotModel<N1> myRobot = new RobotModel<>() {

        @Override
        public Matrix<N1, N1> initial() {
            return null;
        }

        @Override
        public  Matrix<N1, N1> goal() {
            return null;
        }

        @Override
        public boolean goal( Matrix<N1, N1> config) {
            return false;
        }

        @Override
        public boolean clear( Matrix<N1, N1> config) {
            return true;
        }

        @Override
        public boolean link( Matrix<N1, N1> a,  Matrix<N1, N1> b) {
            return true;
        }

    };

    // this test doesn't use the state, just the graph path lengths.
    Matrix<N1,N1> x = new Matrix<>(Nat.N1(),Nat.N1(),  new double[] { 0 });

    @Test
    void testRewiring() {
        Node<N1> root = new Node<>(x);
        assertNull(root.getIncoming());

        Node<N1> node1 = new Node<>(x);
        LinkInterface<N1> link1 = Graph.newLink(root, node1, 1);
        assertEquals(root, node1.getIncoming().get_source());
        assertEquals(1, link1.get_linkDist(), 0.1);
        assertEquals(1, link1.get_pathDist(), 0.1);

        Node<N1> node2 = new Node<>(x);
        LinkInterface<N1> link2 = Graph.newLink(node1, node2, 1);

        assertEquals(node1, node2.getIncoming().get_source());
        assertEquals(1, link2.get_linkDist(), 0.1);
        assertEquals(2, link2.get_pathDist(), 0.1);

        Node<N1> node3 = new Node<>(x);
        LinkInterface<N1> link3 = Graph.newLink(node2, node3, 1);
        assertEquals(node2, node3.getIncoming().get_source());
        assertEquals(1, link3.get_linkDist(), 0.1);
        assertEquals(3, link3.get_pathDist(), 0.1);

        Node<N1> node4 = new Node<>(x);
        LinkInterface<N1> link4 = Graph.newLink(node3, node4, 1);
        assertEquals(node3, node4.getIncoming().get_source());
        assertEquals(1, link4.get_linkDist(), 0.1);
        assertEquals(4, link4.get_pathDist(), 0.1);

        Node<N1> node5 = new Node<>(x);
        LinkInterface<N1> link5 = Graph.newLink(node4, node5, 1);
        assertEquals(node4, node5.getIncoming().get_source());
        assertEquals(1, link5.get_linkDist(), 0.1);
        assertEquals(5, link5.get_pathDist(), 0.1);

        // rewire an inside node
        Graph.rewire(myRobot, node2, node4, 1);

        // the pathdist of that node is updated
        LinkInterface<N1> newLink4 = node4.getIncoming();
        assertEquals(node2, newLink4.get_source());
        assertEquals(1, newLink4.get_linkDist(), 0.1);
        assertEquals(3, newLink4.get_pathDist(), 0.1);

        // make sure the subtree is also updated
        LinkInterface<N1> newLink5 = node5.getIncoming();
        assertEquals(node4, newLink5.get_source());
        assertEquals(1, newLink5.get_linkDist(), 0.1);
        assertEquals(4, newLink5.get_pathDist(), 0.1);

    }

}
