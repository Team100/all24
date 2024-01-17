package frc.robot;

import org.team100.lib.commands.arm.CartesianManualArm;
import org.team100.lib.commands.arm.CartesianManualPositionalArm;
import org.team100.lib.commands.arm.ManualArm;
import org.team100.lib.commands.arm.Sequence;
import org.team100.lib.commands.simple.SimpleManual;
import org.team100.lib.commands.simple.SimpleManualMode;
import org.team100.lib.hid.ControlFactory;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.motion.arm.ArmFactory;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.motion.simple.SimpleSubsystem;
import org.team100.lib.motion.simple.SimpleSubsystemFactory;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Command m_auton;
  private final ArmSubsystem m_armSubsystem;
  private final ArmKinematics m_armKinematicsM;
  private final SimpleSubsystem m_elevator;

  public RobotContainer() {
    ControlFactory controlFactory = new ControlFactory();
    OperatorControl operatorControl = controlFactory.getOperatorControl();

    ///////////////////////
    //
    // ARM
    //

    m_armSubsystem = ArmFactory.get();
    m_armKinematicsM = new ArmKinematics(0.93, 0.92);

    operatorControl.doSomething().whileTrue(new Sequence(m_armSubsystem, m_armKinematicsM));

    operatorControl.never().whileTrue(
        new ManualArm(
            m_armSubsystem,
            operatorControl::lower,
            operatorControl::upper));

    operatorControl.never().whileTrue(
        new CartesianManualArm(
            m_armSubsystem,
            m_armKinematicsM,
            operatorControl::lower,
            operatorControl::upper));

    m_armSubsystem.setDefaultCommand(
        new CartesianManualPositionalArm(
            m_armSubsystem,
            m_armKinematicsM,
            operatorControl::lower,
            operatorControl::upper));

    ///////////////////////////
    //
    // ELEVATOR
    //
    m_elevator = new SimpleSubsystemFactory().get();
    SimpleManualMode simpleMode = new SimpleManualMode();
    m_elevator.setDefaultCommand(new SimpleManual(simpleMode, m_elevator, operatorControl::elevator));

    m_auton = new Sequence(m_armSubsystem, m_armKinematicsM);

  }

}
