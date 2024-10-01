package org.team100.lib.logging;

/**
 * Identifies the importance of an event, so we can filter out the unimportant
 * ones.
 */
public enum Level {
    /**
     * Minimal, curated set of measurements for competition matches.
     */
    COMP(1),
    /**
     * Keep this set small enough to avoid overrunning. This should only contain
     * things you're actively working on.
     */
    DEBUG(2),
    /**
     * Shows absolutely everything. Slow. Overruns. Don't expect normal robot
     * behavior in this mode.
     */
    TRACE(3);

    private int priority;

    private Level(int priority) {
        this.priority = priority;
    }

    public boolean admit(Level other) {
        return this.priority >= other.priority;
    }
}