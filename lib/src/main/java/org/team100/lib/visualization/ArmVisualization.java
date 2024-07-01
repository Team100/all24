package org.team100.lib.visualization;

import java.util.Optional;

import org.team100.lib.async.Async;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmSubsystem;

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

    public static void make(ArmSubsystem armSubsystem, Async async) {
        ArmVisualization v = new ArmVisualization(armSubsystem);
        async.addPeriodic(v::viz, 0.1, "ArmVisualization" + armSubsystem.getName());
    }

    private ArmVisualization(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;

        Optional<ArmAngles> angles = m_armSubsystem.getPosition();

        m_mechanism = new Mechanism2d(100, 100);

        MechanismRoot2d root = m_mechanism.getRoot("SideRoot", 50, 50);

        m_boomLigament = new MechanismLigament2d("Boom",
                25,
                angles.isPresent() ? boomAngleDeg(angles.get()) : 0,
                5, new Color8Bit(Color.kWhite));

        m_stickLigament = new MechanismLigament2d("Stick",
                25,
                angles.isPresent() ? stickAngleDeg(angles.get()) : 0,
                5, new Color8Bit(Color.kLightGreen));

        root.append(m_boomLigament).append(m_stickLigament);

        SmartDashboard.putData("SideView", m_mechanism);
    }

    private void viz() {
        Optional<ArmAngles> angles = m_armSubsystem.getPosition();
        if (angles.isEmpty())
            return;
        m_boomLigament.setAngle(boomAngleDeg(angles.get()));
        m_stickLigament.setAngle(stickAngleDeg(angles.get()));
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
