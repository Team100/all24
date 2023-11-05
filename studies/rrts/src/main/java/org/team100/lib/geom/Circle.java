package org.team100.lib.geom;

import java.awt.Color;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;


public class Circle implements Obstacle {
    private final double _x;
    private final double _y;
    private final double _r;
    private final Ellipse2D.Double _shape;
    private final Color _color;

    public Circle(Color color, double x, double y, double r) {
        _color = color;
        _shape = new Ellipse2D.Double(x - r, y - r, r*2, r*2);
        _x = x;
        _y = y;
        _r = r;
    }

    @Override
    public Shape shape() {
        return _shape;
    }

    @Override
    public Color color() {
        return _color;
    }

    @Override
    public double distToPoint(double x, double y) {
        double dx = x - _x;
        double dy = y - _y;
        return Math.sqrt(dx * dx + dy * dy) - _r;
    }

    @Override
    public double distToSeg(double x1, double y1, double x2, double y2) {
        return Line2D.ptSegDist(x1, y1, x2, y2, _x, _y) - _r;
    }
}
