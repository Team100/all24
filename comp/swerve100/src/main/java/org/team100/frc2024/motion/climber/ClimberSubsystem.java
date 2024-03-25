package org.team100.frc2024.motion.climber;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private final CANSparkFlex s1;
    private final CANSparkFlex s2;
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

                s1 = new CANSparkFlex(leftClimberID, MotorType.kBrushless);
                s2 = new CANSparkFlex(rightClimberID, MotorType.kBrushless);

                s2.setInverted(false);
                s1.setInverted(true);
                
                s1.setSmartCurrentLimit(40);
                s2.setSmartCurrentLimit(40);

                // @vasili 3/25
                // s1 = ServoFactory.neoVortexDistanceServo(m_name + "Left Climber", leftClimberID, true, kCurrentLimit,
                //        m_params, controller,
                //        FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
                // s2 = ServoFactory.neoVortexDistanceServo(m_name + "Right Climber", rightClimberID, false, kCurrentLimit,
                //        m_params, controller,
                //        FeedforwardConstants.makeNeoVortex(), new PIDConstants(1));
                break;
            case BLANK:
            default:
            s1 = new CANSparkFlex(60, MotorType.kBrushless);
            s2 = new CANSparkFlex(61, MotorType.kBrushless);
            s2.setInverted(true);

        }
        // m_viz = new SimpleVisualization(m_name, this);
    }

    public void setLeftWithSoftLimits(double value){
        if(s1.getEncoder().getPosition() > 300){
            if(value >= 0){
                s1.set(0);
                return;
            }
        }

        if(s1.getEncoder().getPosition() < 5){
            if(value <= 0){
                s1.set(0);
                return;
            }
        }

        // s1.set(value);

        Telemetry.get().log(Level.DEBUG, m_name, "LEFT VALUE", value);

    }

    public void setRightWithSoftLimits(double value){
        if(s2.getEncoder().getPosition() > 300){
            if(value >= 0){
                s2.set(0);
                return;
            }
        }


        if(s2.getEncoder().getPosition() < 5){
            if(value <= 0){
                s2.set(0);
                return;
            }
        }

        // s2.set(value);
        Telemetry.get().log(Level.DEBUG, m_name, "RIGHT VALUE", value);


    }


    public void zeroClimbers(){
        s1.getEncoder().setPosition(0);
        s2.getEncoder().setPosition(0);

    }
    public void setLeft(double value){
        s1.set(value);
    }

    public void setRight(double value){
        s2.set(value);
    }


    public double getRightPosition(){
        return s2.getEncoder().getPosition();
    }

    public double getLeftPosition(){
        return s1.getEncoder().getPosition();
    }
    
    @Override
    public void periodic() {
        
        // s1.set(-1);
        // s2.set(-1);

        Telemetry.get().log(Level.DEBUG, m_name, "CLIMBER 1 ENCODER", s1.getEncoder().getPosition());
        Telemetry.get().log(Level.DEBUG, m_name, "CLIMBER 2 ENCODER", s2.getEncoder().getPosition());

        
        Telemetry.get().log(Level.DEBUG, m_name, "current (A) CLIMVER 1", s1.getOutputCurrent());

        Telemetry.get().log(Level.DEBUG, m_name, "current (A) CLIMBER 2", s2.getOutputCurrent());

        Telemetry.get().log(Level.DEBUG, m_name, "DUTY CYCLE 1", s1.getAppliedOutput());

        Telemetry.get().log(Level.DEBUG, m_name, "DUTY CYCLE 2", s2.getAppliedOutput());

        Telemetry.get().log(Level.DEBUG, m_name, "RPM 1", s1.getEncoder().getVelocity());

        Telemetry.get().log(Level.DEBUG, m_name, "RPM 2", s2.getEncoder().getVelocity());

    }

    @Override
    public String getGlassName() {
        return "Climber";
    }
}
