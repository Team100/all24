package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Direct-drive shooter with top and bottom drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DrumShooter extends Shooter {
    // TODO: tune the current limit
    private static final int kCurrentLimit = 40;

    /**
     * Muzzle velocity of game piece exiting the shooter.
     * 
     * The shooter should do whatever is necessary to achieve this;
     * a good approximation for a sticky shooter is the surface
     * speed of whatever wheels are contacting the game piece,
     * but there are many factors that affect the relationship in
     * the real world.
     */
    private static final double kMuzzleVelocityM_S = 30;
    private final String m_name;
    private final LimitedVelocityServo<Distance100> topRoller;
    private final LimitedVelocityServo<Distance100> bottomRoller;
    private final SpeedingVisualization m_viz;
    private final PIDController m_shooterPIDController;

    public DrumShooter(int topRollerID, int bottomRollerID) {
        m_name = Names.name(this);
        m_shooterPIDController = new PIDController(0.0001, 0, 0);
        SmartDashboard.putData("Shooter Controller", m_shooterPIDController);

        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                1,
                0.1,
                30,
                40,
                -40);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
            //TODO tune kV
                topRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Top",
                        topRollerID,
                        false,
                        kCurrentLimit,
                        params,
                        0.122,
                        m_shooterPIDController);
                bottomRoller = ServoFactory.limitedNeoVelocityServo(
                        m_name + "/Bottom",
                        bottomRollerID,
                        true,
                        kCurrentLimit,
                        params,
                        0.122,
                        m_shooterPIDController);
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
    public void forward() {
        topRoller.setVelocity(kMuzzleVelocityM_S);
        bottomRoller.setVelocity(kMuzzleVelocityM_S);
    }

    @Override
    public void stop() {
        topRoller.stop();
        bottomRoller.stop();
    }

    @Override
    public void periodic() {
        topRoller.periodic();
        bottomRoller.periodic();
        m_viz.periodic();
    }

    public boolean readyToShoot() {
        //TODO get real values here
        return topRoller.getVelocity() > 30 && bottomRoller.getVelocity() > 30;
    }

    @Override
    public double getVelocity() {
        return (topRoller.getVelocity() + bottomRoller.getVelocity()) / 2;
    }
}