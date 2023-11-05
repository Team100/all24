package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class DriveManually extends Command {
    private static final boolean fieldRelative = true;
    private final Drivetrain m_swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;

    public DriveManually(Drivetrain m_swerve, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
        this.m_swerve = m_swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        addRequirements(m_swerve);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("drive manually end");
        super.end(interrupted);
    }

    @Override
    public void initialize() {
        System.out.println("drive manually init");
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        System.out.println("drive manually is finished");
        return super.isFinished();
    }

    @Override
    public void execute() {
        System.out.println("drive manually execute");
        final var x = xSpeed.getAsDouble() * Drivetrain.kMaxSpeed;
        final var y = ySpeed.getAsDouble() * Drivetrain.kMaxSpeed;
        final var rot = rotSpeed.getAsDouble() * Drivetrain.kMaxAngularSpeed;
        Pose2d currentPose = m_swerve.getPose();
        Twist2d twist = new Twist2d(x, y, rot);
        SwerveState state = Drivetrain.incremental(currentPose, twist);
        m_swerve.setDesiredState(state);
        //m_swerve.drive(x, y, rot, fieldRelative);
    }
}
