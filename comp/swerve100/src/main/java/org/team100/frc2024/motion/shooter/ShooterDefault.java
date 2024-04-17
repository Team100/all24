package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterDefault extends Command {
    private final Shooter m_shooter;
    private final SwerveDriveSubsystem m_drive;

    public ShooterDefault(Shooter shooter, SwerveDriveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        switch (RobotState100.getShooterState()) {
            case DEFAULTSHOOT:
                execute_shoot();
                break;
            case TEST:
                execute_test();
                break;
            case STOP:
                execute_stop();
                break;
            default:
                // this never happens
                break;
        }

    }

    private void execute_shoot() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_shooter.forward();
            Translation2d robotLocation = m_drive.getPose().getTranslation();
            Translation2d speakerLocation = ShooterUtil.getSpeakerTranslation(alliance.get());
            double distance = robotLocation.getDistance(speakerLocation);
            m_shooter.setAngle(ShooterUtil.getAngleRad(distance));
        }
    }

    private void execute_test() {
        m_shooter.forward();
        m_shooter.setAngle(0.445);
    }

    private void execute_stop() {
        m_shooter.stop();
    }
}
