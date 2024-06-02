package org.team100.util;

public class Arg {
    public static void nonnull(Object o) {
        if (o == null)
            throw new IllegalArgumentException();
    }
}
