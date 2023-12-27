package org.team100.lib.util;

import org.team100.lib.telemetry.TelemetryLevelChooser;
import org.team100.lib.telemetry.Telemetry.Level;

public class Util {

    public static boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }

    /**
     * Print to the console if telemetry level is DEBUG.
     */
    public static void println(String s) {
        if (TelemetryLevelChooser.get().getSelected() == Level.DEBUG)
            System.out.println(s);
    }

    /**
     * Print to the console if telemetry level is DEBUG.
     */
    public static void printf(String s, Object... args) {
        if (TelemetryLevelChooser.get().getSelected() == Level.DEBUG)
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
