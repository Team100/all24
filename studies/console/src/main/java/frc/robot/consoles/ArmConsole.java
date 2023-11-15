package frc.robot.consoles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ArmConsole extends BaseConsole {
    /**
     * Just to show how to wire things up.
     */
    public static class FakeTwoJointArm implements Subsystem {
        /**
         * Operates the arm motors completely manually; this is an override but
         * also the default command.
         */
        public void manual(double boom, double stick) {
        }

        /**
         * Uses trapezoid trajectories and inverse kinematics to put the effector
         * at the specified location.
         */
        public void toGoal(double height, double distance) {
        }

        /**
         * True if the current location is the goal location. This would be an
         * example of an observable.
         */
        public boolean atGoal() {
            return false;
        }

        /**
         * Sets the speed for future trajectories.
         */
        public void setSpeed(double speed) {
        }

        /**
         * Get the speed for observers.
         */
        public double getSpeed() {
            return 0.0;
        }

        /**
         * Override everything and stop.
         */
        public void stop() {
        }
    }

    public static class Config {
        double slowSpeed = 0.25;
        double medSpeed = 0.5;
        double fastSpeed = 1.0;
        double highGoalHeight = 2.0;
        double highGoalDistance = 0.25;
        double lowGoalHeight = 0.5;
        double lowGoalDistance = 0.25;
        double farGoalHeight = 0.5;
        double farGoalDistance = 1.0;
        double notifierRate = 0.1;
    }

    private final FakeTwoJointArm m_fakeArm;
    private final Config m_config;

    public ArmConsole(Config config, FakeTwoJointArm fakeArm) {
        super(portFromName("Arm"));
        m_config = config;
        m_fakeArm = fakeArm;

        // manual control knobs
        m_fakeArm.setDefaultCommand(
                new RunCommand(
                        () -> m_fakeArm.manual(boomKnob(), stickKnob()), m_fakeArm));

        // speed buttons
        new Trigger(() -> stopButton()).whileTrue(new InstantCommand(
                () -> m_fakeArm.stop(), m_fakeArm));
        new Trigger(() -> slowButton()).onTrue(new InstantCommand(
                () -> m_fakeArm.setSpeed(config.slowSpeed), m_fakeArm));
        new Trigger(() -> medButton()).onTrue(new InstantCommand(
                () -> m_fakeArm.setSpeed(config.medSpeed), m_fakeArm));
        new Trigger(() -> fastButton()).onTrue(new InstantCommand(
                () -> m_fakeArm.setSpeed(config.fastSpeed), m_fakeArm));

        // goal setting buttons
        new Trigger(() -> highGoalButton()).whileTrue(new InstantCommand(
                () -> m_fakeArm.toGoal(config.highGoalHeight, config.highGoalDistance), m_fakeArm));
        new Trigger(() -> lowGoalButton()).whileTrue(new InstantCommand(
                () -> m_fakeArm.toGoal(config.lowGoalHeight, config.lowGoalDistance), m_fakeArm));
        new Trigger(() -> farGoalButton()).whileTrue(new InstantCommand(
                () -> m_fakeArm.toGoal(config.farGoalHeight, config.farGoalDistance), m_fakeArm));

        // observer for indicator lights
        new NotifierCommand(
                () -> observe(), config.notifierRate).schedule();
    }

    private boolean highGoalButton() {
        return getRawButton(4);
    }

    private boolean lowGoalButton() {
        return getRawButton(5);
    }

    private boolean farGoalButton() {
        return getRawButton(6);
    }

    // speed control buttons

    private boolean stopButton() {
        return getRawButton(0);
    }

    private boolean slowButton() {
        return getRawButton(1);
    }

    private boolean medButton() {
        return getRawButton(2);
    }

    private boolean fastButton() {
        return getRawButton(3);
    }

    // manual control knobs

    private double boomKnob() {
        return getRawAxis(0);
    }

    private double stickKnob() {
        return getRawAxis(1);
    }

    // output bits:
    // 0: goal light (on/off)
    // 1,2,3: speed lights (one of five states: one of four or none)

    private void setGoalLight(boolean state) {
        applyOutput(state ? 1 : 0, 1, 0);
    }

    private void setSpeedLight(int speed) {
        applyOutput(speed, 3, 1);
    }

    /*
     * Encodes some state in some outputs.
     */
    private void observe() {
        setGoalLight(m_fakeArm.atGoal());
        // speed selector button lights
        if (almostEqual(m_fakeArm.getSpeed(), 0.0)) {
            setSpeedLight(0);
        } else if (almostEqual(m_fakeArm.getSpeed(), m_config.slowSpeed)) {
            setSpeedLight(1);
        } else if (almostEqual(m_fakeArm.getSpeed(), m_config.medSpeed)) {
            setSpeedLight(2);
        } else if (almostEqual(m_fakeArm.getSpeed(), m_config.fastSpeed)) {
            setSpeedLight(3);
        } else {
            setSpeedLight(4);
        }
        sendOutputs();
    }

    private boolean almostEqual(double a, double b) {
        return Math.abs(a - b) < 0.1;
    }
}
