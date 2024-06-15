package org.team100.lib.visualization;

import org.team100.lib.async.AsyncFactory;
import org.team100.lib.motion.simple.Speeding;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Visualization for 1-dof velocity.
 */
public class SpeedingVisualization {
    private static final double kScale = 10.0;
    private final Speeding m_subsystem;
    private final Mechanism2d m_sideView;
    private final MechanismLigament2d m_ligament;

    public static void make(String name, Speeding subsystem) {
        SpeedingVisualization v = new SpeedingVisualization(name, subsystem);
        AsyncFactory.get().addPeriodic(v::viz, 0.1, "SpeedingVisualization" + name);
    }

    private SpeedingVisualization(String name, Speeding subsystem) {
        m_subsystem = subsystem;
        double position = m_subsystem.getVelocity();
        m_sideView = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_sideView.getRoot("root", 50, 50);
        m_ligament = new MechanismLigament2d(name, kScale * position, 90, 5, new Color8Bit(Color.kOrange));
        root.append(m_ligament);
        SmartDashboard.putData(name, m_sideView);
    }

    private void viz() {
        double position = m_subsystem.getVelocity();
        m_ligament.setLength(kScale * position);
    }

}
