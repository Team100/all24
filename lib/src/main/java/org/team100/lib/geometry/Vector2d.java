package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;

public class Vector2d {

    private final double m_x;
    private final double m_y;
    private final Rotation2d m_theta;

    public Vector2d(double x, double y) {
        m_x = x;
        m_y = y;
        m_theta = new Rotation2d(Math.atan(m_x / m_y));
    }

    public Vector2d(double hypo, Rotation2d theta) {
        m_x = hypo * Math.sin(theta.getRadians());
        m_y = hypo * Math.cos(theta.getRadians());
        m_theta = theta;
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }

    public Rotation2d getTheta() {
        return m_theta;
    }

    public static Vector2d add(Vector2d v1, Vector2d v2) {
        double x = v1.getX() + v2.getX();
        double y = v1.getY() + v2.getY();
        return new Vector2d(x, y);
    }

    public static Vector2d sub(Vector2d v1, Vector2d v2) {
        double x = v1.getX() - v2.getX();
        double y = v1.getY() - v2.getY();
        return new Vector2d(x, y);
    }

    public static Vector2d fromAdjacentSide(double adjacentSide, Rotation2d theta) {
        double m_x = Math.atan(theta.getRadians()) * adjacentSide;
        double m_y = adjacentSide;
        return new Vector2d(m_x, m_y);
    }

}
