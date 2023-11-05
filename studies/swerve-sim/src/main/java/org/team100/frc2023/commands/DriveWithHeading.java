// adapted from main2023 on july 9th 2023
package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class DriveWithHeading extends Command {
    /** Creates a new DrivePID. */
    Drivetrain m_robotDrive;
    ProfiledPIDController m_headingController;
    Supplier<Rotation2d> m_desiredRotation;
    // ChassisSpeeds targetChasisSpeeds = new ChassisSpeeds();

    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;
    private Rotation2d lastRotationSetpoint;

    private boolean snapMode = false;

    private static final double kSpeedModifier = 5;
    Pose2d currentPose;
    // Supplier<Double> rotSpeed;
    double thetaOuput = 0;
    double yOutput;
    double thetaControllerOutput;

    public DriveWithHeading(Drivetrain robotDrive, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            Supplier<Rotation2d> desiredRotation, DoubleSupplier rotSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_robotDrive = robotDrive;
        m_headingController = m_robotDrive.headingController;
        m_desiredRotation = desiredRotation;

        lastRotationSetpoint = new Rotation2d(0);

        yOutput = 0;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;

        // m_headingController.setTolerance(0.1);
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        currentPose = m_robotDrive.getPose();

        SmartDashboard.putData("Drive with Heading:", this);

        addRequirements(m_robotDrive);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSwitch = MathUtil.applyDeadband(xSpeed.getAsDouble(), .05);
        double ySwitch = MathUtil.applyDeadband(ySpeed.getAsDouble(), .05);
        double rotSwitch = MathUtil.applyDeadband(rotSpeed.getAsDouble(), .05);
        // double xDBRemoved = (xSwitch - .1 * Math.signum(xSwitch)) / .9;
        // double yDBRemoved = (ySwitch - .1 * Math.signum(ySwitch)) / .9;
        // double rotDBRemoved = (rotSwitch - .1 * Math.signum(rotSwitch)) / .9;

        double xDBRemoved = xSwitch;
        double yDBRemoved = ySwitch;
        double rotDBRemoved = rotSwitch;

        // Set desired rotation to POV (if pressed) or else last setpoint
        double desiredRotation = lastRotationSetpoint.getRadians();
        Rotation2d pov = m_desiredRotation.get();
        if (pov != null) {
            snapMode = true;
            desiredRotation = pov.getRadians();
        }
        currentPose = m_robotDrive.getPose();
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        if (snapMode && Math.abs(rotSwitch) < 0.1) {
            thetaControllerOutput = m_headingController.calculate(currentRads, desiredRotation);
            // System.out.println(thetaControllerOutput);
            // thetaOuput = MathUtil.clamp(thetaControllerOutput*kSpeedModifier + m_headingController.getSetpoint().velocity, -kSpeedModifier, kSpeedModifier);
            thetaOuput = thetaControllerOutput*kSpeedModifier + m_headingController.getSetpoint().velocity;

        } else {
            snapMode = false;
            thetaOuput = (rotDBRemoved/2)*kSpeedModifier;
            desiredRotation = currentRads;
        }

        // if(m_arm.getLowerArm() >=0.2){
            m_robotDrive.driveWithHeading(xDBRemoved * kSpeedModifier, yDBRemoved * kSpeedModifier, thetaOuput, true);
        // } else {
            // m_robotDrive.driveWithHeading(xDBRemoved * kSpeedModifier, yDBRemoved * kSpeedModifier, thetaOuput, true);

        // }

        lastRotationSetpoint = new Rotation2d(desiredRotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        snapMode = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> m_headingController.getPositionError(), null);
        builder.addDoubleProperty("Measurment", () -> currentPose.getRotation().getRadians(), null);
        builder.addDoubleProperty("Setpoint Position", () -> m_headingController.getSetpoint().position, null);
        builder.addDoubleProperty("Setpoint Velocity", () -> m_headingController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Goal", () -> m_headingController.getGoal().position, null);
        builder.addDoubleProperty("HeadingControllerOutput", () -> thetaControllerOutput, null);
        // builder.addDoubleProperty("ControllerOutput", () -> radiansPerSecond, null);
        // builder.addDoubleProperty("Chassis Speeds", () ->
        // targetChasisSpeeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Theta Outpit", () -> thetaOuput, null);

    }
}
