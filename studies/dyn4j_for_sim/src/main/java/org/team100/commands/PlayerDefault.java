package org.team100.commands;

import org.team100.robot.RobotAssembly;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** The "player" robot is manually controlled. */
public class PlayerDefault extends Command {
    private static final double kForce = 200;
    private static final double kTorque = 100;

    private final XboxController m_control;
    private final RobotAssembly m_player;

    public PlayerDefault(RobotAssembly player) {
        m_control = new XboxController(0);
        m_player = player;
        addRequirements(player.getRobotSubsystem());
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

        if (m_control.getRawButton(1)) {
            m_player.intake();
        }
        if (m_control.getRawButton(2)) {
            m_player.outtake();
        }
        if (m_control.getRawButton(3)) {
            m_player.shoot();
        }
        if (m_control.getRawButton(4)) {
            m_player.lob();
        }
        if (m_control.getRawButton(5)) {
            m_player.amp();
        }
        if (m_control.getRawButton(6)) {
            m_player.rotateToShoot();
        }
    }

}
