package org.team100.robot;

import java.util.Random;

import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Source extends SubsystemBase {
    private static final double kStdDev = 0.25;
    private final SimWorld m_world;
    private final Translation2d m_target;
    private final Random m_random;

    private int m_notes;

    /** provide the location the notes will appear. */
    public Source(SimWorld world, Translation2d target) {
        m_world = world;
        m_target = target;
        m_random = new Random();
        m_notes = 45;
    }

    /** Feed a note through the source if there are any left. */
    public void feed() {
        if (m_notes > 0) {
            m_world.addNote(
                    m_target.getX() + m_random.nextGaussian(0, kStdDev),
                    m_target.getY() + m_random.nextGaussian(0, kStdDev));
            m_notes--;
        }
    }

    public Translation2d getTarget() {
        return m_target;
    }

}
