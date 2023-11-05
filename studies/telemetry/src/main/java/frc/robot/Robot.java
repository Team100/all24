package frc.robot;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private final Telemetry t = Telemetry.get();

    @Override
    public void robotPeriodic() {
        long now = RobotController.getFPGATime();
        t.log("/time", now);
    }
}
