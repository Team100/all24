package org.team100.lib.util;

public record Tolerance(
        double kTranslationTolerance,
        double kVelocityTolerance,
        double kAngularTolerance) {
}
