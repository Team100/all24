package frc.robot;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An attempt to make absolute rotational control possible, using
 * a PID controller on the rotation axis. Same thing on the other axes
 * just to see.
 */
@SuppressWarnings("unused")
public class DriveManuallyWithPID extends Command {
    private static final double dt = 0.02;
    private static final double xSpeed = 1.0;
    private static final double ySpeed = 1.0;
    private static final double thetaSpeed = 1.0;
    private final Drivetrain m_swerve;
    private final XboxController m_controller;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private Pose2d setpoint;

    public DriveManuallyWithPID(Drivetrain drivetrain) {
        m_swerve = drivetrain;
        m_controller = new XboxController(0);
        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);
        thetaController = new PIDController(1, 0, 0);
        setpoint = m_swerve.getPose();
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        setpoint = m_swerve.getPose();
    }

    @Override
    public void execute() {
        // for now, velocity inputs:
        double xInput = -1.0 * m_controller.getRightY();
        double yInput = -1.0 * m_controller.getRightX();
        double rotInput = -1.0 * m_controller.getLeftX();

        double newX = setpoint.getX() + xSpeed * dt * xInput;
        double newY = setpoint.getY() + ySpeed * dt * yInput;
        double newTheta = setpoint.getRotation().getRadians() + thetaSpeed * dt * rotInput;

        double xOutput = xController.calculate(newX, setpoint.getX());
        double yOutput = xController.calculate(newY, setpoint.getY());
        double thetaOutput = xController.calculate(newTheta, setpoint.getRotation().getRadians());

        Pose2d currentPose = m_swerve.getPose();
        Twist2d twist = new Twist2d(xOutput, yOutput, thetaOutput);
        SwerveState state = Drivetrain.incremental(currentPose, twist);
        m_swerve.setDesiredState(state);

        //m_swerve.drive(xOutput, yOutput, thetaOutput, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.truncate();
    }

}
