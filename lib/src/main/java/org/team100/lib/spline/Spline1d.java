package org.team100.lib.spline;

/**
 * One-dimensional quintic spline, representing five derivatives of position.
 * 
 * The "t" parameter here is not time, its just a parameter.
 */
public class Spline1d {
    /** crackle */
    private final double a;
    /** snap */
    private final double b;
    /** jerk */
    private final double c;
    /** acceleration */
    private final double d;
    /** velocity */
    private final double e;
    /** position */
    private final double f;

    private Spline1d(double a, double b, double c, double d, double e, double f) {
        if (Double.isNaN(a))
            throw new IllegalArgumentException();
        if (Double.isNaN(b))
            throw new IllegalArgumentException();
        if (Double.isNaN(c))
            throw new IllegalArgumentException();
        if (Double.isNaN(d))
            throw new IllegalArgumentException();
        if (Double.isNaN(e))
            throw new IllegalArgumentException();
        if (Double.isNaN(f))
            throw new IllegalArgumentException();
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
    }

    public static Spline1d newSpline1d(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1) {
        if (Double.isNaN(x0))
            throw new IllegalArgumentException();
        if (Double.isNaN(x1))
            throw new IllegalArgumentException();
        if (Double.isNaN(dx0))
            throw new IllegalArgumentException();
        if (Double.isNaN(dx1))
            throw new IllegalArgumentException();
        if (Double.isNaN(ddx0))
            throw new IllegalArgumentException();
        if (Double.isNaN(ddx1))
            throw new IllegalArgumentException();

        double a = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        double b = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        double c = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        double d = 0.5 * ddx0;
        double e = dx0;
        double f = x0;
        return new Spline1d(a, b, c, d, e, f);
    }

    Spline1d addCoefs(Spline1d other) {
        double aa = a + other.a;
        double bb = b + other.b;
        double cc = c + other.c;
        double dd = d + other.d;
        double ee = e + other.e;
        double ff = f + other.f;
        return new Spline1d(aa, bb, cc, dd, ee, ff);
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    public double getPosition(double t) {
        return a * t * t * t * t * t + b * t * t * t * t + c * t * t * t + d * t * t + e * t + f;
    }

    /**
     * @return rate of change of position with respect to parameter, i.e. ds/dt
     */
    public double getVelocity(double t) {
        return 5 * a * t * t * t * t + 4 * b * t * t * t + 3 * c * t * t + 2 * d * t + e;
    }

    /**
     * @return acceleration of position with respect to parameter, i.e. d^2s/dt^2
     */
    public double getAcceleration(double t) {
        return 20 * a * t * t * t + 12 * b * t * t + 6 * c * t + 2 * d;
    }

    /**
     * @return jerk of position with respect to parameter, i.e. d^3s/dt^3.
     */
    public double getJerk(double t) {
        return 60 * a * t * t + 24 * b * t + 6 * c;
    }

}