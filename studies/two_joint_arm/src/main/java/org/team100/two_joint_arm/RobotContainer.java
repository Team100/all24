package org.team100.two_joint_arm;

import org.team100.two_joint_arm.hid.Control;
import org.team100.lib.config.Identity;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.two_joint_arm.commands.arm.ArmTrajectory;
import org.team100.two_joint_arm.commands.arm.ManualArm;
import org.team100.two_joint_arm.commands.arm.SetConeMode;
import org.team100.two_joint_arm.commands.arm.SetCubeMode;
import org.team100.two_joint_arm.subsystems.arm.ArmFactory;
import org.team100.two_joint_arm.subsystems.arm.ArmInterface;
import org.team100.two_joint_arm.subsystems.arm.ArmPosition;

public class RobotContainer {
    private final Control control;
    private final LEDIndicator m_indicator;
    private final ArmInterface m_arm;

    public RobotContainer() {

        control = new Control() {
        };

        m_indicator = new LEDIndicator(8);
        Identity identity = Identity.get();
        m_arm = new ArmFactory(identity).get();

        m_arm.setDefaultCommand(new ManualArm(m_arm, control::lowerSpeed, control::upperSpeed));
        control.armHigh(new ArmTrajectory(ArmPosition.HIGH, m_arm, false));
        control.armSafe(new ArmTrajectory(ArmPosition.SAFE, m_arm, false));
        control.armSubstation(new ArmTrajectory(ArmPosition.SUB, m_arm, false));
        control.coneMode(new SetConeMode(m_arm, m_indicator));
        control.cubeMode(new SetCubeMode(m_arm, m_indicator));
        control.armLow(new ArmTrajectory(ArmPosition.MID, m_arm, false));
        control.armSafeBack(new ArmTrajectory(ArmPosition.SAFEBACK, m_arm, false));
        control.armToSub(new ArmTrajectory(ArmPosition.SUBTOCUBE, m_arm, false));
        control.safeWaypoint(new ArmTrajectory(ArmPosition.SAFEWAYPOINT, m_arm, false));
        control.oscillate(new ArmTrajectory(ArmPosition.SUB, m_arm, true));
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_arm.close();
    }

}
