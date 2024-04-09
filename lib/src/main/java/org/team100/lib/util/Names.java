package org.team100.lib.util;

import org.team100.lib.dashboard.Glassy;

/** To make the names of things in glass look consistent. */
public class Names {
    /** Create a name for logging, using the class simplename. */
    public static String append(String root, Glassy leaf) {
        return root + "/" + name(leaf);
    }

    public static String name(Glassy obj) {
        return obj.getGlassName();
    }

    private Names() {
        //
    }

}
