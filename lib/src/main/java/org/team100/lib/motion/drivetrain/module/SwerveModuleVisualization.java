package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.telemetry.TelemetryLevelChooser;
import org.team100.lib.telemetry.Telemetry.Level;

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

    /**
     * Module name is used as the root key, so it may not contain a slash.
     */
    public SwerveModuleVisualization(SwerveModule100 module) {
        String name = module.getName();
        if (name.contains("/"))
            throw new IllegalArgumentException();
        m_module = module;
        m_mechanism = new Mechanism2d(100, 100);
        // the root name cannot have slashes in it.
        MechanismRoot2d root = m_mechanism.getRoot(name, 50, 50);
        m_steer = new MechanismLigament2d("steer",
                50, angle(), 3, new Color8Bit(Color.kWhite));
        m_drive = new MechanismLigament2d("drive",
                speed(), angle(), 10, new Color8Bit(Color.kOrange));
        root.append(m_drive);
        root.append(m_steer);
        SmartDashboard.putData("Swerve Viz/" + name, m_mechanism);
    }

    public void periodic() {
        if (TelemetryLevelChooser.get().getSelected() == Level.DEBUG) {
            m_drive.setAngle(angle());
            m_drive.setLength(speed());
            m_steer.setAngle(angle());
        }
    }

    private double angle() {
        return m_module.getPosition().angle.getDegrees();
    }

    private double speed() {
        return m_module.getState().speedMetersPerSecond * 10;
    }
}
