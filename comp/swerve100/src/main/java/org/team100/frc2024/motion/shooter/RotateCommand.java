package org.team100.frc2024.motion.shooter;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateCommand extends Command {
    private static final double kToleranceRad = 0.05;
    private final SwerveDriveSubsystem m_drive;
    private final PIDController m_controller;
    private final Rotation2d m_goal;

    public RotateCommand(SwerveDriveSubsystem drive) {
        m_drive = drive;
        m_controller = new PIDController(2, 0, 0);
        // the goal would normally be passed in the constructor
        m_goal = new Rotation2d(Math.PI / 2);
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty())
            return;
        double measurement = m_drive.getPose().getRotation().getRadians();
        double setpoint = m_goal.getRadians();
        setpoint = Math100.getMinDistance(measurement, setpoint);
        double dtheta = m_controller.calculate(measurement, setpoint);
        FieldRelativeVelocity twist = new FieldRelativeVelocity(0, 0, dtheta);
        m_drive.driveInFieldCoords(twist);
    }

    @Override
    public boolean isFinished() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            Util.warn("no alliance present!");
            return true;
        }
        double errorRad = m_goal.getRadians() - m_drive.getPose().getRotation().getRadians();
        return Math.abs(errorRad) < kToleranceRad;
    }
}
