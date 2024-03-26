package org.team100.frc2024.motion.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterTable {

    public static final ShooterTable instance = new ShooterTable();

    private final InterpolatingDoubleTreeMap m_table;

    public ShooterTable() {
        // loadTable();
        m_table = new InterpolatingDoubleTreeMap();
        loadTable();
    }

    public double getAngle(double m) {
        return m_table.get(m);
    }

    // MORE NUMBERSSDDD
    public void loadTable() {

        m_table.put(1.552869, 0.95);

        m_table.put(2.109692, 0.760000);

        m_table.put(2.479717, 0.700000);

        m_table.put(3.095720, 0.600000);

        m_table.put(3.819086, 0.55);

        m_table.put(4.01, 0.535);

        m_table.put(4.5, 0.5);

        m_table.put(5.0, 0.48);

    }

}
