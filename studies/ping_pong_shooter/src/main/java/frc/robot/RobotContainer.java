package frc.robot;

import org.team100.subsystems.ElevationSubsystem;
import org.team100.subsystems.FeedSubsystem;
import org.team100.subsystems.WheelSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final CommandXboxController m_controller;
    private final ElevationSubsystem m_elev;
    private final WheelSubsystem m_wheels;
    private final FeedSubsystem m_feed;

    public RobotContainer() {
        m_controller = new CommandXboxController(0);
        m_elev = new ElevationSubsystem();
        m_wheels = new WheelSubsystem();
        m_feed = new FeedSubsystem();
    }

    public void bind() {

        m_controller.povUp().whileTrue(m_elev.up());
        m_controller.povDown().whileTrue(m_elev.down());
        m_elev.setDefaultCommand(m_elev.stop());

        m_controller.b().whileTrue(m_wheels.shoot());
        m_wheels.setDefaultCommand(m_wheels.stop());

        m_controller.a().whileTrue(m_feed.feed());
        m_feed.setDefaultCommand(m_feed.stop());
    }

    public void teleopInit() {
        m_elev.init();
    }

}
