package org.team100.commands;

import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** The "player" robot is manually controlled. */
public class PlayerDefaultDrive extends Command {
    private static final double kForce = 200;
    private static final double kTorque = 100;

    private final XboxController m_control;
    private final RobotAssembly m_player;

    public PlayerDefaultDrive(RobotAssembly player, XboxController control) {
        m_control = control;
        m_player = player;
        addRequirements(player.getDriveSubsystem());
    }

    @Override
    public String getName() {
        return "Player Default: " + m_player.getName();
    }

    @Override
    public void execute() {
        double steer = -m_control.getLeftX(); // axis 0
        double driveX = -m_control.getRightY(); // axis 5
        double driveY = -m_control.getRightX(); // axis 4
        m_player.apply(driveX * kForce, driveY * kForce, steer * kTorque);


        if (m_control.getRawButton(3)) {
            m_player.shoot();
        }
        if (m_control.getRawButton(4)) {
            m_player.lob();
        }
        if (m_control.getRawButton(5)) {
            m_player.amp();
        }

    }

}
