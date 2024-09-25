package org.team100.lib.dashboard;

/**
 * Since Glass records NT keys in its layout configuration, we should keep these
 * keys the same, and we should use the same keys for simulation and prod.
 */
public interface Glassy {
    default String getGlassName() {
        return this.getClass().getSimpleName();
    }
}
