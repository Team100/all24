package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auton;
import frc.robot.commands.FieldRelativeDrive;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    public static final int kDriverControllerPort = 0;
    public static final boolean kFieldRelative = true;
    private final XboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    public RobotContainer() {
        m_driverController = new XboxController(kDriverControllerPort);
        m_robotDrive = new DriveSubsystem();

        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
        if (kFieldRelative) {
            m_robotDrive.setDefaultCommand(new FieldRelativeDrive(m_driverController, m_robotDrive));
        } else {
            m_robotDrive.setDefaultCommand(new ManualDrive(m_driverController, m_robotDrive));
        }
    }

    public DriveSubsystem getRobotDrive() {
        return m_robotDrive;
    }

    /** Zeros the outputs of all subsystems. */
    public void zeroAllOutputs() {
        m_robotDrive.tankDriveVolts(0, 0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new Auton(m_robotDrive);
    }
}
