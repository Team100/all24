package org.team100.lib.commands.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.state.State100;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Remembers the most recent desired heading, substituting null if there's any
 * dtheta input.
 */
public class HeadingLatch {
    private static final double unlatch = 0.01;
    private static final double maxARad_S2 = 10;
    private Rotation2d m_desiredRotation = null;

    public Rotation2d latchedRotation(
            State100 state,
            Rotation2d currentRotation,
            Rotation2d pov,
            double inputOmega) {
        if (Math.abs(inputOmega) > unlatch) {
            // if the driver is trying to drive, then let them
            m_desiredRotation = null;
        } else if (pov != null) {
            // if the driver is trying to snap, then let them
            m_desiredRotation = pov;
        } else if (m_desiredRotation == null &&
                Experiments.instance.enabled(Experiment.StickyHeading)) {
            // if the driver is providing no input, and there's no sticky heading,
            // then use the current heading as the sticky heading.
            // give the robot a chance to slow down to avoid overshoot.
            double t = Math.abs(state.v()) / maxARad_S2;
            // System.out.printf("t %6.3f\n", t);
            m_desiredRotation = new Rotation2d(state.x() + state.v() * t / 2);
        }
        return m_desiredRotation;
    }

    public void unlatch() {
        m_desiredRotation = null;
    }
}
