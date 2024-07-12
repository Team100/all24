package org.team100.lib.visualization;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.module.SwerveModule100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Displays the module steering and velocity as a mechanism.
 */
public class SwerveModuleVisualization {
    private final SwerveModule100 m_module;
    private final Mechanism2d m_mechanism;
    private final MechanismLigament2d m_steer;
    private final MechanismLigament2d m_drive;

    public SwerveModuleVisualization(SwerveModule100 module) {
        // the glass visualization requires the key to be a root key
        // so eliminate the slashes.
        String name = module.getName().replace("/", "_");
        m_module = module;
        m_mechanism = new Mechanism2d(100, 100);
        // the root name cannot have slashes in it.
        MechanismRoot2d root = m_mechanism.getRoot(name, 50, 50);
        m_steer = new MechanismLigament2d("steer",
                50, 0, 3, new Color8Bit(Color.kWhite));
        m_drive = new MechanismLigament2d("drive",
                0, 0, 10, new Color8Bit(Color.kOrange));
        root.append(m_drive);
        root.append(m_steer);
        SmartDashboard.putData("Swerve Viz/" + name, m_mechanism);
    }

    public void viz() {
        Optional<Rotation2d> angle = m_module.getPosition().angle;
        if (Telemetry.get().getLevel().admit(Level.TRACE) && angle.isPresent()) {
            m_drive.setAngle(angle.get().getDegrees());
            m_drive.setLength(m_module.getState().speedMetersPerSecond * 10);
            m_steer.setAngle(angle.get().getDegrees());
        }
    }


}
