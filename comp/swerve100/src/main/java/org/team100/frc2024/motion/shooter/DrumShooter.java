package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class DrumShooter extends Shooter implements Speeding {
    private final String m_name;
    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SpeedingVisualization m_viz;

    public DrumShooter(int topRollerID, int bottomRollerID) {
        m_name = Names.name(this);

        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                1,
                0.1,
                8,
                30,
                -30);

        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                topRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Top",
                        topRollerID,
                        true,
                        params);
                bottomRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Bottom",
                        bottomRollerID,
                        false,
                        params);
                break;
            case BLANK:
            default:
                topRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top",
                        params);
                bottomRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom",
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void setVelocity(double value) {
        topRoller.setVelocity(value);
        bottomRoller.setVelocity(value);
    }

    public double getFirstRollerVelocity() {
        return topRoller.getVelocity();
    }
    
    public double getSecondRollerVelocity() {
        return bottomRoller.getVelocity();
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