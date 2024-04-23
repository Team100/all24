package org.team100.robot;

import org.team100.commands.Blue;
import org.team100.commands.Red;
import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
    private final SimWorld world;
    /** Friends */
    private final Blue m_blue;
    /** Foes */
    private final Red m_red;

    public RobotContainer() {
        world = new SimWorld();
        m_blue = new Blue(world);
        m_red = new Red(world);
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public void init() {
        m_blue.init();
        m_red.init();
        world.render();
    }

    public void periodic() {
        world.update();
        world.render();
        m_blue.periodic();
        m_red.periodic();
    }
}
