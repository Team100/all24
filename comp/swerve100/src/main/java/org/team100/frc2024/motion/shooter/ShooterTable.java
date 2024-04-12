package org.team100.frc2024.motion.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Interpolates gun elevation in radians, given range in meters. */
public class ShooterTable {

    public static final ShooterTable instance = new ShooterTable();

    private final InterpolatingDoubleTreeMap m_table;

    public ShooterTable() {
        m_table = new InterpolatingDoubleTreeMap();
        loadTable();
    }

    public double getAngleRad(double rangeM) {
        return m_table.get(rangeM);
    }

    public void loadTable() {
        m_table.put(1.49, 0.9);
        m_table.put(2.07, 0.78);
        m_table.put(2.5, 0.66);
        m_table.put(3.02, 0.59);
        m_table.put(3.59, 0.53);
        m_table.put(4.1, 0.475);
        m_table.put(4.5, 0.44);

    }

}
