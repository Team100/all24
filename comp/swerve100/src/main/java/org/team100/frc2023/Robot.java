package org.team100.frc2023;

import java.io.IOException;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private final Telemetry t = Telemetry.get();

    private RobotContainer m_robotContainer;

    public Robot() {
        try {
            m_robotContainer = new RobotContainer();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.get().name());
        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.ledStart();
    }

    @Override
    public void disabledPeriodic() {
        double keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        t.log("/robot/key list size", keyListSize);
        if (keyListSize == 0) {
            m_robotContainer.red();
        } else {
            m_robotContainer.green();
        }
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.cancelAuton();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        m_robotContainer.runTest2();
    }

    @Override
    public void close() {
        super.close();
        m_robotContainer.close();
    }
}