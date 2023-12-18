package org.team100.lib.motion.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Visualization for 2-dof arm.
 * 
 * It's in a separate class to avoid clutter.
 */
public class ArmVisualization {
    private final ArmSubsystem m_armSubsystem;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_boomLigament;
    private final MechanismLigament2d m_stickLigament;

    public ArmVisualization(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;

        ArmAngles angles = m_armSubsystem.getPosition();

        m_mechanism = new Mechanism2d(100, 100);

        MechanismRoot2d root = m_mechanism.getRoot("SideRoot", 50, 50);

        m_boomLigament = new MechanismLigament2d("Boom",
                25, boomAngleDeg(angles), 5, new Color8Bit(Color.kWhite));

        m_stickLigament = new MechanismLigament2d("Stick",
                25, stickAngleDeg(angles), 5, new Color8Bit(Color.kLightGreen));

        root.append(m_boomLigament).append(m_stickLigament);

        SmartDashboard.putData("SideView", m_mechanism);
    }

    public void periodic() {
        ArmAngles angles = m_armSubsystem.getPosition();
        m_boomLigament.setAngle(boomAngleDeg(angles));
        m_stickLigament.setAngle(stickAngleDeg(angles));
    }

    // zero is straight up
    private double boomAngleDeg(ArmAngles angles) {
        return Units.radiansToDegrees(angles.th1) + 90;
    }

    // the viz stick angle is relative to the boom
    private double stickAngleDeg(ArmAngles angles) {
        return Units.radiansToDegrees(angles.th2 - angles.th1);
    }

}
