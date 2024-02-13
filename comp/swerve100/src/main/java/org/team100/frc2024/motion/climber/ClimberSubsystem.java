package org.team100.frc2024.motion.climber;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.motion.simple.SimpleVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Dual winch climber.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.01m => 0.0314 m/turn
 * therefore top speed is around 3 m/s.
 * There's no reason to go so fast though.
 * 
 * Try a reasonable accel value.
 * 
 * TODO: this mechanism can self-destruct without positional limits, so add
 * some.
 * 
 * TODO: add climber to selftest.
 */
public class ClimberSubsystem extends SubsystemBase implements Positioning {
    // TODO: tune the current limit
    private static final int kCurrentLimit = 30;
    private final String m_name;
    private final SysParam m_params;
    private final PositionServoInterface<Distance100> s1;
    private final PositionServoInterface<Distance100> s2;
    private final SimpleVisualization m_viz;
    private final PIDConstants m_velocityPIDConstants;
    private final FeedforwardConstants m_lowLevelFeedforwardConstants;
    private final PIDController m_climberPositionController;


    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        m_velocityPIDConstants = new PIDConstants(0.0001, 0, 0);
        m_lowLevelFeedforwardConstants = new FeedforwardConstants(0.122,0,0.1,0.065);
        m_climberPositionController = new PIDController(1, 0, 0);
        SmartDashboard.putData("Climber PID Positional Controller", m_climberPositionController);


        m_params = new SysParam(
                20.0,
                0.01,
                1,
                3,
                -3);
        switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV
                s1 = ServoFactory.neoVortexDistanceServo(
                        m_name + "/Left",
                        leftClimberID,
                        false,
                        kCurrentLimit,
                        m_params,
                        new PIDController(1, 0, 0),
                        m_lowLevelFeedforwardConstants,
                        m_velocityPIDConstants);
                s2 = ServoFactory.neoVortexDistanceServo(
                        m_name + "/Right",
                        rightClimberID,
                        true,
                        kCurrentLimit,
                        m_params,
                        new PIDController(1, 0, 0),
                        m_lowLevelFeedforwardConstants,
                        m_velocityPIDConstants);
                break;
            case BLANK:
            default:
                s1 = ServoFactory.simulatedDistanceServo(
                        m_name + "/Left",
                        m_params,
                        new PIDController(1, 0, 0));
                s2 = ServoFactory.simulatedDistanceServo(
                        m_name + "/Right",
                        m_params,
                        new PIDController(1, 0, 0));
        }
        m_viz = new SimpleVisualization(m_name, this);
    }

    /** Set velocity in meters per second */
    public void set(double value) {
        s1.setVelocity(value);
        s2.setVelocity(value);
    }

    public void setPosition(double value) {
        s1.setPosition(value);
        s2.setPosition(value);
    }

    @Override
    public double getPositionRad() {
        return (s1.getPosition() + s2.getPosition()) / 2;
    }

    @Override
    public void periodic() {
        s1.periodic();
        s2.periodic();
        m_viz.periodic();
    }
}
