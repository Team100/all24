package frc.robot;

import org.team100.persistent_parameter.PersistentParameter;

import edu.wpi.first.wpilibj.TimedRobot;

/** This just shows that the sim UI and the persistent file both work.  */
public class Robot extends TimedRobot {
    private final PersistentParameter p;

    public Robot() {
        p = new PersistentParameter("foo", 1.0);
    }

    @Override
    public void robotPeriodic() {
        System.out.println(p.get());
    }
}
