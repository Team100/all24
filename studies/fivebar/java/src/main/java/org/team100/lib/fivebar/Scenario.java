package org.team100.lib.fivebar;

/**
 * Details of a particular geometry and motor combination."""
 */
public class Scenario {
    String name;
    // link lengths, meters
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
    // position of P1, meters
    double x1;
    double y1;
    // reduction
    double ratio;
    // stall torque, Nm
    double Tmax;
    // the work envelope
    double w;
    double h;
    double xcenter;
    double ycenter;
    double xmin;
    double xmax;
    double ymin;
    double ymax;

    public double right() {
        return xcenter + w / 2;
    }

    public double left() {
        return xcenter - w / 2;
    }

    public double top() {
        return ycenter + h / 2;
    }

    public double bottom() {
        return ycenter - h / 2;
    }
}
