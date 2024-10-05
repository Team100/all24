package org.team100.lib.util;

public class Arg {
    /** Throws if o is null. */
    public static void nonnull(Object o) {
        if (o == null)
            throw new IllegalArgumentException();
    }

    /** Throws if o is null or empty. */
    public static void nonempty(Object[] o) {
        if (o == null)
            throw new IllegalArgumentException();
        if (o.length == 0)
            throw new IllegalArgumentException();
    }

    private Arg() {
        //
    }
}
