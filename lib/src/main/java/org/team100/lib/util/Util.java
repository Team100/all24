package org.team100.lib.util;

public class Util {
    public static boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }

    /**
     * This is here to centralize all the printing I want to keep.
     * TODO: respect telemetry level.
     */
    public static void println(String s) {
        System.out.println(s);
    }

    /**
     * This is here to centralize all the printing I want to keep.
     * TODO: respect telemetry level.
     */
    public static void printf(String s, Object... args) {
        System.out.printf(s, args);
    }

    public static void warn(String s) {
        System.out.println("WARNING: " + s);
    }

    private Util() {
        //
    }
}
