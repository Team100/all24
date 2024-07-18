package org.team100.lib.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Depending on the alliance, return the red option, or the blue option, or
 * empty if there is no alliance.
 * 
 * You could also inline this whole thing.
 */
public class AllianceSelector<T> implements Supplier<Optional<T>> {

    private final T m_red;
    private final T m_blue;

    public AllianceSelector(T red, T blue) {
        m_red = red;
        m_blue = blue;
    }

    @Override
    public Optional<T> get() {
        return DriverStation.getAlliance().map(
                x -> switch (x) {
                    case Red -> m_red;
                    case Blue -> m_blue;
                });
    }

}
