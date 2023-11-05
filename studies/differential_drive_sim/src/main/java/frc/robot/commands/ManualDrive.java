package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Single joystick, using the robot reference frame. Y axis is linear velocity,
 * X axis is angular velocity.
 */
public class ManualDrive extends Command {
    private final XboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    public ManualDrive(XboxController driverController, DriveSubsystem robotDrive) {
        m_driverController = driverController;
        m_robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {
        // nothing
    }

    @Override
    public void execute() {
        m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getLeftX());
    }

    @Override
    public void end(boolean interrupted) {
        // nothing
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
