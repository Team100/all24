package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class FlywheelShooter extends Shooter implements Speeding {
    private final String m_name;
    private final LimitedVelocityServo<Distance100> leftShooter;
    private final LimitedVelocityServo<Distance100> rightShooter;
    private final SpeedingVisualization m_viz;

    public FlywheelShooter(int leftShooterID, int rightShooterID) {
        m_name = Names.name(this);
        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                20,
                10,
                8,
                30,
                -30);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                leftShooter = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Left",
                        leftShooterID,
                        false,
                        params);
                rightShooter = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Right",
                        rightShooterID,
                        false,
                        params);
                break;
            case BLANK:
            default:
                leftShooter = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Left",
                        params);

                rightShooter = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Right",
                        params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void setVelocity(double value) {
        // leftShooter.setVelocity(value);
        // rightShooter.setVelocity(value);
        
    }

    @Override
    public double getVelocity() {
        return (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2;
    }

    @Override
    public void periodic() {
        leftShooter.periodic();
        rightShooter.periodic();
        m_viz.periodic();
    }
}
