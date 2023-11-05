package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Single joystick, using the field reference frame.
 */
public class FieldRelativeDrive extends Command {
    public static class Control {
        public final double x;
        public final double y;

        public Control(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private final XboxController m_driverController;
    private final DriveSubsystem m_robotDrive;

    public FieldRelativeDrive(XboxController driverController, DriveSubsystem robotDrive) {
        m_driverController = driverController;
        m_robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    /**
     * x and y refer to field coords from driver perspective, so x is straight
     * ahead, y is to the left
     */
    public static Control rotate(double headingDeg, double x, double y) {
        double headingRad = Math.toRadians(headingDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double fwd = x * cos + y * sin;
        double rot = -x * sin + y * cos;
        return new Control(fwd, rot);
    }

    @Override
    public void initialize() {
        // nothing
    }

    @Override
    public void execute() {
        // note x/y reversal
        Control control = rotate(
                m_robotDrive.getHeading(),
                -m_driverController.getLeftY(),
                -m_driverController.getLeftX());
        m_robotDrive.arcadeDrive(control.x, control.y);
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
