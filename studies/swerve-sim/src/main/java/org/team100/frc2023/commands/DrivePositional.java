package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

/** positional xy control instead of velocity control */
public class DrivePositional extends Command {
    private final Drivetrain m_robotDrive;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final Supplier<Rotation2d> rot;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotController;
    private double initialX;
    private double initialY;
    private Pose2d initialPose;

    public DrivePositional(Drivetrain robotDrive, DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> rot) {
        m_robotDrive = robotDrive;
        this.x = x;
        this.y = y;
        this.rot = rot;
        xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2, 4));
        yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2, 4));
        rotController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6, 12));
    }

    @Override
    public void initialize() {
        initialPose = m_robotDrive.getPose();
        initialX = x.getAsDouble();
        initialY = y.getAsDouble();
    }

    @Override
    public void execute() {
        double desiredX = x.getAsDouble() - initialX;
        double desiredY = y.getAsDouble() - initialY;
        double desiredRot = rot.get().getRadians();

        double currentX = m_robotDrive.getPose().getX() - initialPose.getX();
        double currentY = m_robotDrive.getPose().getY() - initialPose.getY();
        double currentRot = MathUtil.angleModulus(m_robotDrive.getPose().getRotation().getRadians());

        double xControllerOutput = xController.calculate(currentX, desiredX);
        double yControllerOutput = yController.calculate(currentY, desiredY);
        double rotControllerOutput = rotController.calculate(currentRot, desiredRot);

        double xOutput = xControllerOutput + xController.getSetpoint().velocity;
        double yOutput = yControllerOutput + yController.getSetpoint().velocity;
        double rotOutput = rotControllerOutput + rotController.getSetpoint().velocity;

        m_robotDrive.driveWithHeading(xOutput, yOutput, rotOutput, true);
    }

}
