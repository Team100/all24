package org.team100.lib.spline;

/**
 * One-dimensional quintic spline, representing five derivatives of position.
 * 
 * The "t" parameter here is not time, its just a parameter.
 */
public class Spline1d {
    /** crackle */
    double a;
    /** snap */
    double b;
    /** jerk */
    double c;
    /** acceleration */
    double d;
    /** velocity */
    double e;
    /** position */
    double f;

 public Spline1d(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1) {
     a = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
     b = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
     c = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
     d = 0.5 * ddx0;
     e = dx0;
     f = x0;
 }

 public Spline1d(Spline1d other) {
     a = other.a;
     b = other.b;
     c = other.c;
     d = other.d;
     e = other.e;
     f = other.f;
 }

    Spline1d addCoefs(Spline1d other) {
        Spline1d ret = new Spline1d(this);
        ret.a += other.a;
        ret.b += other.b;
        ret.c += other.c;
        ret.d += other.d;
        ret.e += other.e;
        ret.f += other.f;
        return ret;
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