package org.team100.robot;

import org.team100.alliance.Alliance;
import org.team100.alliance.Blue;
import org.team100.alliance.Red;
import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
    private final SimWorld world;
    /** Friends */
    private final Alliance m_blue;
    /** Foes */
    private final Alliance m_red;

    public RobotContainer() {
        world = new SimWorld();
        m_blue = new Blue(world);
        m_red = new Red(world);
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public void robotInit() {
        m_blue.reset();
        m_red.reset();
        world.render();
    }

    public void teleopInit() {
        m_blue.begin();
        m_red.begin();
        world.render();
    }

    public void teleopExit() {
        m_blue.reset();
    }

    public void periodic() {
        world.update();
        world.render();
        m_blue.periodic();
        m_red.periodic();
    }
}
