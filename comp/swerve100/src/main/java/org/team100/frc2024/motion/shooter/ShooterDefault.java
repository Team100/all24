package org.team100.frc2024.motion.shooter;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefault extends Command {
    private static final double kThreshold = 8;

    private final Shooter m_shooter;
    private final SwerveDriveSubsystem m_drive;
    private final Supplier<Double> m_pivotUpSupplier;
    private final Supplier<Double> m_pivotDownSupplier;

    public ShooterDefault(
            Shooter shooter,
            SwerveDriveSubsystem drive,
            Supplier<Double> pivotUpSupplier,
            Supplier<Double> pivotDownSupplier) {
        m_shooter = shooter;
        m_drive = drive;
        m_pivotUpSupplier = pivotUpSupplier;
        m_pivotDownSupplier = pivotDownSupplier;
        SmartDashboard.putNumber("Shooter Angle", 0.2); // 22.5a
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;

        double distance = m_drive.getPose().getTranslation()
                .getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
        switch (RobotState100.getRobotState()) {
            case SHOOTING:

                switch (RobotState100.getShooterState()) {
                    case DEFAULTSHOOT:
                        m_shooter.forward();
                        // m_shooter.setAngleWithOverride(ShooterUtil.getAngle(m_drive.getPose().getX()),
                        // m_pivotUpSupplier.get(), m_pivotDownSupplier.get()); //22.5a
                        // m_shooter.setAngle(SmartDashboard.getNumber("Shooter Angle", 0.2)); //22.5a
                        // m_shooter.setAngle(0.4);
                        m_shooter.setAngle(ShooterUtil.getAngle(distance));

                        break;
                    case TEST:
                        // m_shooter.setAngle(Smart/Dashboard.getNumber("Shooter Angle", 0.2)); //22.5a
                        m_shooter.forward();
                        m_shooter.setAngle(0.9);
                        break;
                    case STOP:
                        m_shooter.stop();
                        break;
                    case LOB:
                        m_shooter.forward();
                        m_shooter.setAngle(0.6);
                    default:
                        m_shooter.setDutyCycle(0);
                        break;

                }
                break;

            case AMPING:
                // m_shooter.stop();
                break;
            case NONE:
                // m_shooter.stop();
                break;
            default:
                break;
            // m_shooter.stop();
            // m_shooter.setAngle(0.0);

        }

    }
}
