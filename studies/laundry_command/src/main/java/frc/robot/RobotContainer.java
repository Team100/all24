package frc.robot;

import org.team100.frc2023.autonomous.Autonomous;
import org.team100.frc2023.commands.Manual;
import org.team100.frc2023.control.LaundryStick;
import org.team100.frc2023.subsystems.LaundryArm;
import org.team100.frc2023.subsystems.LaundryDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    // TODO: try lower gear ratios
    private static final int kGearRatio = 125;
    private static final double kDriveSpeed = 1.0;
    private static final double kDriveTime = 2.0;
    private final LaundryDrive m_drive;
    private final LaundryArm m_arm;
    private final Command m_auton;

    public RobotContainer() {
        m_drive = new LaundryDrive(getDifferentialDrive());
        m_arm = new LaundryArm(getArmController(), getArmMotor(), kGearRatio);
        m_drive.setDefaultCommand(new Manual(new LaundryStick(new Joystick(0)), m_arm, m_drive));
        m_auton = new Autonomous(m_drive, kDriveSpeed, kDriveTime, m_arm);
    }

    public void scheduleAuton() {
        m_auton.schedule();
    }

    public void cancelAuton() {
        m_auton.cancel();
    }

    private DifferentialDrive getDifferentialDrive() {
        Talon leftMotor = new Talon(0);
        Talon rightMotor = new Talon(1);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        return new DifferentialDrive(leftMotor, rightMotor);
    }

    private ProfiledPIDController getArmController() {
        return new ProfiledPIDController(
                10, // P
                0, // I
                0, // D
                new Constraints(
                        50, // max velocity (infinite)
                        20)); // max accel (infinite)
    }

    private CANSparkMax getArmMotor() {
        CANSparkMax motor = new CANSparkMax(2, MotorType.kBrushless);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(10);
        motor.setIdleMode(IdleMode.kCoast);
        return motor;
    }

}
