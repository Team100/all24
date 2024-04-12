package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefault extends Command {
    private final Shooter m_shooter;
    private final SwerveDriveSubsystem m_drive;

    public ShooterDefault(Shooter shooter, SwerveDriveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        SmartDashboard.putNumber("Shooter Angle", 0.2);
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return;

        Translation2d robotLocation = m_drive.getPose().getTranslation();
        Translation2d speakerLocation = ShooterUtil.getSpeakerTranslation(alliance.get());
        double distance = robotLocation.getDistance(speakerLocation);
        switch (RobotState100.getRobotState()) {
            case SHOOTING:
                switch (RobotState100.getShooterState()) {
                    case DEFAULTSHOOT:
                        m_shooter.forward();
                        // m_shooter.setAngle(SmartDashboard.getNumber("Shooter Angle", 0.2));
                        m_shooter.setAngle(ShooterUtil.getAngleRad(distance));
                        break;
                    case TEST:
                        m_shooter.forward();
                        // m_shooter.setAngle(Smart/Dashboard.getNumber("Shooter Angle", 0.2));
                        m_shooter.setAngle(0.445);
                        break;
                    case STOP:
                        m_shooter.stop();
                        break;
                    case LOB:
                        m_shooter.forward();
                        m_shooter.setAngle(0.6);
                        break;
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
