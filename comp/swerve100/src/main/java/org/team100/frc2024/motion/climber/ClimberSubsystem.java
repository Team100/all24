package org.team100.frc2024.motion.climber;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.drive.NeoVortexDriveMotor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
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
    private static final int kCurrentLimit = 40;
    private final String m_name;
    private final SysParam m_params;
    private final PositionServo<Distance100> s1;
    private final PositionServo<Distance100> s2;

    public ClimberSubsystem(int leftClimberID, int rightClimberID) {
        m_name = Names.name(this);
        // TODO tune Feedforward and PID, put real gear ratio, and real motor if motor
        // is different
        m_params = new SysParam(
                20.0,
                0.1,
                1,
                3,
                -3);
        PIDController controller = new PIDController(1, 0, 0);

        switch (Identity.instance) {
            case COMP_BOT:
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

    public void setLeftWithSoftLimits(double value) {
        if (s1.getPosition() > 1) {
            if (value >= 0) {
                s1.setDutyCycle(0);
                return;
            }
        }

        if (s1.getPosition() < 0) {
            if (value <= 0) {
                s1.setDutyCycle(0);
                return;
            }
        }

        s1.setDutyCycle(value);

        Telemetry.get().log(Level.DEBUG, m_name, "LEFT VALUE", value);

    }

    public void setRightWithSoftLimits(double value) {
        if (s2.getPosition() > 1) {
            if (value >= 0) {
                s2.setDutyCycle(0);
                return;
            }
        }

        if (s2.getPosition() < 0) {
            if (value <= 0) {
                s2.setDutyCycle(0);
                return;
            }
        }
        s2.setDutyCycle(value);
        Telemetry.get().log(Level.DEBUG, m_name, "RIGHT VALUE", value);

    }

    public void setLeft(double value) {
        s1.setPosition(value);
    }

    public void setRight(double value) {
        s2.setDutyCycle(value);
    }

    public void setClimbGO() {
        //TODO get real up pose
        s1.setPosition(2);
        s2.setPosition(2);
    }

    public void rest() {
        s1.setPosition(0);
        s2.setPosition(0);
    }

    @Override
    public String getGlassName() {
        return "Climber";
    }

}
