package org.team100.commands;

public record Tolerance(
        double kTranslationTolerance,
        double kVelocityTolerance,
        double kAngularTolerance) {
}
