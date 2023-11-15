package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Prints the state of joystick inputs. */
public class JoystickDumper extends CommandBase {
    private final Joystick m_joystick;

    public JoystickDumper(Joystick joystick) {
        m_joystick = joystick;
    }

    @Override
    public void initialize() {
        doit();
    }

    @Override
    public final boolean isFinished() {
        return true;
    }

    public void doit() {
        System.out.printf(
                "axes: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                ax(0), ax(1), ax(2), ax(3), ax(4), ax(5), ax(6), ax(7), ax(8));
        System.out.printf(
                "buttons: %b  %b %b %b %b %b %b %b"
                        + " %b %b %b %b %b %b %b %b"
                        + " %b %b %b %b %b %b %b %b"
                        + " %b %b %b %b %b %b %b %b\n",
                bu(1), bu(2), bu(3), bu(4), bu(5), bu(6), bu(7), bu(8),
                bu(9), bu(10), bu(11), bu(12), bu(13), bu(14), bu(15), bu(16),
                bu(17), bu(18), bu(19), bu(20), bu(21), bu(22), bu(23), bu(24),
                bu(25), bu(26), bu(27), bu(28), bu(29), bu(30), bu(31), bu(32));

        // m_joystick.setOutput(0, true);
    }

    private double ax(int a) {
        return m_joystick.getRawAxis(a);
    }

    private boolean bu(int b) {
        return m_joystick.getRawButton(b);
    }
}
