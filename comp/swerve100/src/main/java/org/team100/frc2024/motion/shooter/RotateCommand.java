package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateCommand extends Command {
    private final SwerveDriveSubsystem m_drive;

    public RotateCommand(SwerveDriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent())
            return;
        PIDController controller = new PIDController(2, 0, 0);
        double measurement = m_drive.getPose().getRotation().getRadians();
        double setpoint = new Rotation2d(Math.PI/2).getRadians();

        setpoint = Math100.getMinDistance(measurement, setpoint);

        double value = controller.calculate(m_drive.getPose().getRotation().getRadians(), setpoint);
        Twist2d twist = new Twist2d(0, 0, value);
        m_drive.driveInFieldCoords(twist, 0.02);
    }

    @Override
    public boolean isFinished() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            Util.warn("no alliance present!");
            return true;
        }
        return Math.abs(ShooterUtil.getRobotRotationToSpeaker(
                alliance.get(),
                m_drive.getPose().getTranslation(), 0).getRadians()
                - m_drive.getPose().getRotation().getRadians()) < 0.05;
    }
}
