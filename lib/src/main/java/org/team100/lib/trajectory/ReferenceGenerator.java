package org.team100.lib.trajectory;

/** Produces references for trajectory tracking. */
public interface ReferenceGenerator<Reference> {
    public interface Sample<Reference> {
        /**
         * The current measurement; this is included because it may have been used to
         * create the reference (e.g. applying a differential), so including it here
         * allows the consumer to avoid any noise of remeasurement.
         */
        Reference measurement();

        /** The desired state. */
        Reference reference();
    }

    Sample<Reference> sample(double time);

}
