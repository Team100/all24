package org.team100.frc2024.motion.climber;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
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
public class ClimberSubsystem extends SubsystemBase implements Glassy {
    // TODO: tune the current limit
    private static final int kCurrentLimit = 2;
    private final String m_name;
    private final SysParam m_params;
    private final PositionServo<Distance100> s1;
    private final PositionServo<Distance100> s2;
    PIDController controller;
    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        controller = new PIDController(1, 0, 0);


        m_params = new SysParam(
                20.0,
                0.01,
                1,
                3,
                -3);
        switch (Identity.instance) {
            case COMP_BOT:
            //TODO tune kV
s1 = ServoFactory.neoVortexDistanceServo(m_name + "Left Climber", leftClimberID, true, kCurrentLimit,
                        m_params, controller,
                        FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
                s2 = ServoFactory.neoVortexDistanceServo(m_name + "Right Climber", rightClimberID, false, kCurrentLimit,
                        m_params, controller,
                        FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
                break;
            case BLANK:
            default:
            s1 = ServoFactory.neoVortexDistanceServo(m_name + "Left Climber", leftClimberID, true, kCurrentLimit,
            m_params, controller,
            FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
    s2 = ServoFactory.neoVortexDistanceServo(m_name + "Right Climber", rightClimberID, false, kCurrentLimit,
            m_params, controller,
            FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
        }
        // m_viz = new SimpleVisualization(m_name, this);
    }

    public void setLeftWithSoftLimits(double value){
        if(s1.getPosition() > 300){
            if(value >= 0){
                s1.set(0);
                return;
            }
        }

        if(s1.getPosition() < 5){
            if(value <= 0){
                s1.set(0);
                return;
            }
        }

        // s1.set(value);

        Telemetry.get().log(Level.DEBUG, m_name, "LEFT VALUE", value);

    }

    public void setRightWithSoftLimits(double value){
        if(s2.getPosition() > 300){
            if(value >= 0){
                s2.set(0);
                return;
            }
        }


        if(s2.getPosition() < 5){
            if(value <= 0){
                s2.set(0);
                return;
            }
        }

        // s2.set(value);

        Telemetry.get().log(Level.DEBUG, m_name, "RIGHT VALUE", value);

    }

    public void setLeft(double value){
        s1.set(value);
    }

    public void setRight(double value){
        s2.set(value);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
    public void setClimbGO() {
        //TODO get real up pose
        s1.setPositionDirect(2);
        s2.setPositionDirect(2);
    }

    public void rest() {
        s1.setPositionDirect(0);
        s2.setPositionDirect(0);
    }
    
}
