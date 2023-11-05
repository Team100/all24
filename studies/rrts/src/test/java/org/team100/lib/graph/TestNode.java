package org.team100.lib.graph;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.planner.RobotModel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class TestNode {

    RobotModel<N2> myRobot = new RobotModel<>() {

        @Override
        public Matrix<N2, N1> initial() {
            return null;
        }

        @Override
        public Matrix<N2, N1> goal() {
            return null;
        }

        @Override
        public boolean goal(Matrix<N2,N1> config) {
            return false;
        }

        @Override
        public boolean clear(Matrix<N2,N1> config) {
            return true;
        }

        @Override
        public boolean link(Matrix<N2,N1> a, Matrix<N2,N1> b) {
            return true;
        }

    };

    @Test
    void testfoo() {
        Node<N2> root = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 0, 0 }));
        assertArrayEquals(new double[] { 0, 0 }, root.getState().getData());
        assertNull(root.getIncoming());

        Node<N2> node1 = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 1, 0 }));
        LinkInterface<N2> link1 = Graph.newLink(root, node1, 1);
        assertArrayEquals(new double[] { 1, 0 }, node1.getState().getData());
        assertEquals(root, node1.getIncoming().get_source());
        assertEquals(1, link1.get_linkDist(), 0.1);
        assertEquals(1, link1.get_pathDist(), 0.1);

        Node<N2> node2 = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 2, 0 }));
        LinkInterface<N2> link2 = Graph.newLink(node1, node2, 1);
        assertArrayEquals(new double[] { 2, 0 }, node2.getState().getData());
        assertEquals(node1, node2.getIncoming().get_source());
        assertEquals(1, link2.get_linkDist(), 0.1);
        assertEquals(2, link2.get_pathDist(), 0.1);

        // now add a second child to the root. what happens?
        Node<N2> node3 = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 0, 1 }));
        LinkInterface<N2> link3 = Graph.newLink(root, node3, 1);
        assertArrayEquals(new double[] { 0, 1 }, node3.getState().getData());
        assertEquals(root, node3.getIncoming().get_source());
        assertEquals(1, link3.get_linkDist(), 0.1);
        assertEquals(1, link3.get_pathDist(), 0.1);

        // where does the third child go?
        Node<N2> node4 = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 0, -1 }));
        LinkInterface<N2> link4 = Graph.newLink(root, node4, 1);
        assertArrayEquals(new double[] { 0, -1 }, node4.getState().getData());
        assertEquals(root, node4.getIncoming().get_source());
        assertEquals(1, link4.get_linkDist(), 0.1);
        assertEquals(1, link4.get_pathDist(), 0.1);

        Node<N2> node5 = new Node<>(new Matrix<>(Nat.N2(),Nat.N1(),new double[] { 1, 1 }));
        LinkInterface<N2> link5 = Graph.newLink(node1, node5, 1);
        assertArrayEquals(new double[] { 1, 1 }, node5.getState().getData());
        assertEquals(node1, node5.getIncoming().get_source());
        assertEquals(1, link5.get_linkDist(), 0.1);
        assertEquals(2, link5.get_pathDist(), 0.1);
    }

}
