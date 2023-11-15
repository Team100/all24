package frc.robot.consoles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PilotConsole extends BaseConsole {

    public static class FakeDrivetrain implements Subsystem {
        public void drive(double x, double y, double rot) {
        }
        public void setSpeed(double speed) {
        }
        public void toGoalOne() {
        }
    }

    public static class Config {
        double fastSpeed = 1.0;
        double notifierRate = 0.1;
    }

    private final FakeDrivetrain m_fakeDrivetrain;
    private final Config m_config;

    public PilotConsole(Config config, FakeDrivetrain fakeDrivetrain) {
        super(portFromName("Pilot"));
        m_config = config;
        m_fakeDrivetrain = fakeDrivetrain;
        m_fakeDrivetrain.setDefaultCommand(new RunCommand(
            () -> m_fakeDrivetrain.drive(xAxis(), yAxis(), rotation()), m_fakeDrivetrain));
        
        new Trigger(() -> fastButton()).onTrue(new InstantCommand(
            ()->m_fakeDrivetrain.setSpeed(config.fastSpeed), m_fakeDrivetrain
        ));

        // indicator light notifier
        new NotifierCommand(
                () -> observe(), m_config.notifierRate).schedule();
    }

    // manual axes
    private double xAxis() {
        return getRawAxis(0);
    }
    private double yAxis() {
        return getRawAxis(1);
    }
    private double rotation() {
        return getRawAxis(2);
    }

    // buttons
    private boolean fastButton() {
        return getRawButton(0);
    }

    /*
     * Encodes the state in some of the outputs
     */
    private void observe() {
    }
}
