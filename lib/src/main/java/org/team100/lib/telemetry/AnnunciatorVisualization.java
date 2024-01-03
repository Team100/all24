package org.team100.lib.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Shows the annunicator status as a blob in glass. */
public class AnnunciatorVisualization {
    private static final Color8Bit kOn = new Color8Bit(Color.kRed);
    private static final Color8Bit kIndeterminate = new Color8Bit(Color.kYellow);
    private static final Color8Bit kOff = new Color8Bit(Color.kBlack);
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_ligament;

    public AnnunciatorVisualization() {
        m_mechanism = new Mechanism2d(100, 100);
        MechanismRoot2d root = m_mechanism.getRoot("root", 0, 50);
        m_ligament = new MechanismLigament2d("light", 100, 0, 100, kIndeterminate);
        root.append(m_ligament);
        SmartDashboard.putData("annunciator", m_mechanism);
    }

    public void set(boolean value) {
        if (value) {
            m_ligament.setColor(kOn);
        } else {
            m_ligament.setColor(kOff);
        }
    }
}
