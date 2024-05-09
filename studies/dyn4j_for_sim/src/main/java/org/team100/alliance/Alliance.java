package org.team100.alliance;

/** Alliance coordinates the behavior of the member robots. */
public interface Alliance {
    /**
     * Reset robot locations.
     * State machines to initial state.
     */
    void reset();

    /**
     * State machines to first active state.
     */
    void begin();

    void periodic();
}
