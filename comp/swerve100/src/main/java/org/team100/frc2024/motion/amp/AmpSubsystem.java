package org.team100.frc2024.motion.amp;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.AngularVisualization;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A 1-dof arm driven by two separate motors with opposite phases.
 */
public class AmpSubsystem extends SubsystemBase implements Positioning {
    // ALERT! notice this very high current limit!!  ALERT! 
    private static final int kCurrentLimit = 80;

    private final String m_name;
    private final SysParam m_params;
    private final PositionServoInterface<Angle100> ampAngleServo;
    private final AngularVisualization m_viz;
    private final PIDConstants m_armPositionConstants; 
    private final PIDConstants m_armVelocityPIDConstants; 
    private final FeedforwardConstants m_lowLevelFeedforwardConstants;



    public AmpSubsystem(int pivotID) {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                45,
                9,
                5)
                ;

        m_armPositionConstants = new PIDConstants(2.5, 0.1, 0);
        m_lowLevelFeedforwardConstants = new FeedforwardConstants(0.122,0,0.1,0.065);
        m_armVelocityPIDConstants = new PIDConstants(0.0001, 0, 0);



        switch (Identity.instance) {
            case COMP_BOT:
                //TODO tune kV
                ampAngleServo = ServoFactory.neoAngleServo(
                        m_name + "/Left",
                        pivotID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        m_params,
                        m_armPositionConstants, //2.5 0.1
                        m_lowLevelFeedforwardConstants,
                        m_armVelocityPIDConstants); //Where did this come from?
                break;
            case BLANK:
            default:
                ampAngleServo = ServoFactory.simulatedAngleServo(
                        m_name + "/Left",
                        m_params,
                        new PIDController(1, 0, 0));
                
        }
        m_viz = new AngularVisualization(m_name, this);
    }

    /**
     * Set angle relative to the zero.
     * 
     * TODO: calibrate to the horizontal, reset the actual angle at the stop,
     * and/or use an absolute encoder.
     * 
     * @param value
     */
    public void setAmpPosition(double value) {
        ampAngleServo.setPosition(value);
    }



    public void stop() {
        ampAngleServo.stop();
    }

    @Override
    public double getPositionRad() {
        // return (ampAngleServoRight.getPosition() + ampAngleServoLeft.getPosition()) / 2;
        return ampAngleServo.getPosition();
    }

    public boolean inPosition() {
        // TODO get real values here
        return getPositionRad() < 0.75 * Math.PI && getPositionRad() > .5 * Math.PI;
    }

    @Override
    public void periodic() {
        ampAngleServo.periodic();
        m_viz.periodic();
    }
}
