package org.team100.lib.util;

public class Util {

    public static boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }

    /** This exists to make it clear which println statements to keep. */
    public static void println(Object s) {
        System.out.println(s);
    }

    public static void printf(String s, Object... args) {
        System.out.printf(s, args);
    }

    /**
     * Print to the console regardless of telemetry level.
     */
    public static void warn(String s) {
        System.out.println("WARNING: " + s);
    }

    /** Throw if x is out of range. This is a more strict version of "clamp" :-) */
    public static double inRange(double x, double minX, double maxX) {
        if (x < minX)
            throw new IllegalArgumentException(String.format("arg is %f which is below %f", x, minX));
        if (x > maxX)
            throw new IllegalArgumentException(String.format("arg is %f which is above %f", x, maxX));
        return x;
    }

    public static double notNaN(double x) {
        if (Double.isNaN(x))
            throw new IllegalArgumentException("arg is NaN");
        return x;
    }

    private Util() {
        //
    }
}
