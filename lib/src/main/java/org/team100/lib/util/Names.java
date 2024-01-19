package org.team100.lib.util;

/** To make the names of things in glass look consistent. */
public class Names {
    /** Create a name for logging, using the class simplename. */
    public static String append(String root, Object leaf) {
        return root + "/" + name(leaf);
    }

    public static String name(Object obj) {
        return name(obj.getClass());
    }

    public static String name(Class<?> clazz) {
        return clazz.getSimpleName();
    }

    private Names() {
        //
    }

}
