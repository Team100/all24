package frc.robot.consoles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimbConsole extends BaseConsole {

    /** just for showing how to do it */
    public static class FakeClimber implements Subsystem {
        public void manual(double up, double tilt) {
        }

        public void toGoalOne() {
        }

        public boolean atGoal() {
            return false;
        }
    }

    public static class Config {
        double notifierRate = 0.1;
    }

    private final FakeClimber m_fakeClimber;
    private final Config m_config;

    public ClimbConsole(Config config, FakeClimber fakeClimber) {
        super(portFromName("Climb"));
        m_config = config;
        m_fakeClimber = fakeClimber;
        // manual control knobs
        m_fakeClimber.setDefaultCommand(
                new RunCommand(
                        () -> m_fakeClimber.manual(upKnob(), tiltKnob()), m_fakeClimber));

        // goal setting button
        new Trigger(() -> goalOneButton()).whileTrue(new InstantCommand(
                () -> m_fakeClimber.toGoalOne(), m_fakeClimber));

        // indicator light notifier
        new NotifierCommand(
                () -> observe(), m_config.notifierRate).schedule();

    }

    // manual control knobs

    private double upKnob() {
        return getRawAxis(0);
    }

    private double tiltKnob() {
        return getRawAxis(1);
    }

    // goal button

    private boolean goalOneButton() {
        return getRawButton(0);
    }

    // output bits:
    // 0: goal light (on/off)

    private void setGoalLight(boolean state) {
        applyOutput(state ? 1 : 0, 1, 0);
    }

    /*
     * Encodes the state in some of the outputs
     */
    private void observe() {
        setGoalLight(m_fakeClimber.atGoal());
        sendOutputs();
    }
}
