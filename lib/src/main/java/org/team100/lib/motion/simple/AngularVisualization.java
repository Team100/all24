package org.team100.lib.motion.simple;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Visualization for a 1-dof arm.
 */
public class AngularVisualization {
    private final Positioning m_subsystem;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_boomLigament;

    public AngularVisualization(String name, Positioning subsystem) {
        m_subsystem = subsystem;
        m_mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_mechanism.getRoot("SideRoot", 50, 50);
        double position = m_subsystem.getPositionRad();
        m_boomLigament = new MechanismLigament2d("Boom",
                25, position, 5, new Color8Bit(Color.kWhite));
        root.append(m_boomLigament);
        SmartDashboard.putData(name, m_mechanism);
    }

    public void periodic() {
        double positionDeg = Units.radiansToDegrees(m_subsystem.getPositionRad());
        m_boomLigament.setAngle(positionDeg);
    }
}
