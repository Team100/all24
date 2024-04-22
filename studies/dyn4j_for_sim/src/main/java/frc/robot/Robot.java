package frc.robot;

import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final SimWorld simWorld;

    public Robot() {
        simWorld = new SimWorld();
    }

    @Override
    public void robotInit() {
        simWorld.render();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // reset position
        simWorld.init();
    }

    @Override
    public void teleopPeriodic() {
        simWorld.update();
        simWorld.render();
        simWorld.behavior();
    }
}
