package org.team100.lib.util;

/**
 * Global debug output control. This is for development and testing only, to
 * produce a whole lot of printed output. Never use this in comp!
 */
public class Debug {
    private static final boolean kEnable = true;

    public static boolean enable() {
        return kEnable;
    }

    private Debug() {
        //
    }

}
