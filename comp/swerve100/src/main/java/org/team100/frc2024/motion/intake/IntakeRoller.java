package org.team100.frc2024.motion.intake;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

/**
 * TODO: add intake to selftest.
 */
public class IntakeRoller extends Intake implements Speeding {
    private final String m_name;
    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SpeedingVisualization m_viz;

    public IntakeRoller(int topCAN, int bottomCAN) {
        m_name = Names.name(this);
        SysParam rollerParameter = SysParam.limitedNeoVelocityServoSystem(
                1,
                1,
                5,
                5,
                -5);

        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                topRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Top Roller",
                        topCAN,
                        false,
                        rollerParameter);
                bottomRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Bottom Roller",
                        bottomCAN,
                        false,
                        rollerParameter);
                break;
            case BLANK:
            default:
                topRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top Roller",
                        rollerParameter);
                bottomRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom Roller",
                        rollerParameter);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void setIntake(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
        m_viz.periodic();
    }

    @Override
    public double getVelocity() {
        return (topRoller.getVelocity() + bottomRoller.getVelocity()) / 2;
    }
}
