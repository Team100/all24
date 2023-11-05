package org.team100.lib.graph;

import edu.wpi.first.math.Num;

/** Allows experimentation with link behaviors. */
public interface LinkInterface<States extends Num> {

    // Path path();

    Node<States> get_source();

    Node<States> get_target();

    double get_linkDist();

    void set_PathDist(double d);

    double get_pathDist();

}
