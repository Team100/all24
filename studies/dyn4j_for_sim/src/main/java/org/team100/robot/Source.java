package org.team100.robot;

import java.util.Random;

import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Source extends SubsystemBase {
    private static final double kStdDev = 0.25;
    private final SimWorld m_world;
    private final double m_x;
    private final double m_y;
    private final Random m_random;

    private int m_notes;

    /** provide the location the notes will appear. */
    public Source(SimWorld world, double x, double y) {
        m_world = world;
        m_x = x;
        m_y = y;
        m_random = new Random();
        m_notes = 45;
    }

    /** Feed a note through the source if there are any left. */
    public void feed() {
        if (m_notes > 0) {
            m_world.addNote(
                    m_x + m_random.nextGaussian(0, kStdDev),
                    m_y + m_random.nextGaussian(0, kStdDev));
            m_notes--;
        }
    }

}
