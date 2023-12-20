package org.team100.lib.util;

public class Util {
    public static boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }
}
