package org.team100.robot;

import java.util.function.BooleanSupplier;

import org.team100.commands.PlayerDefaultDrive;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** This robot is controlled by a human. */
public class RealPlayerAssembly extends RobotAssembly {

    private final XboxController m_control;

    public RealPlayerAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        m_control = new XboxController(0);
        m_drive.setDefaultCommand(new PlayerDefaultDrive(m_drive, m_control));
        whileTrue(() -> m_control.getRawButton(1), m_indexer.run(m_indexer::intake));
        whileTrue(() -> m_control.getRawButton(2), m_indexer.run(m_indexer::outtake));
        whileTrue(() -> m_control.getRawButton(3), Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::shoot)));
        whileTrue(() -> m_control.getRawButton(4), Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::lob)));
        whileTrue(() -> m_control.getRawButton(5), Commands.parallel(
                m_indexer.run(m_indexer::towardsShooter),
                m_shooter.run(m_shooter::shoot)));
        whileTrue(() -> m_control.getRawButton(6), m_drive.run(m_drive::rotateToShoot));
    }

    @Override
    public boolean isNPC() {
        return false;
    }

    ///////////////////////////////////////////

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
