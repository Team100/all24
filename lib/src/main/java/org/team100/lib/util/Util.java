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
    public static void println(String s) {
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

    private Util() {
        //
    }
}
