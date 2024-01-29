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
    private final PositionServoInterface<Angle100> ampAngleServoLeft;
    private final PositionServoInterface<Angle100> ampAngleServoRight;
    private final AngularVisualization m_viz;
    private final PIDController m_armPositionPIDController; 
    private final PIDConstants m_armVelocityPIDConstants; 
    private final FeedforwardConstants m_lowLevelFeedforwardConstants;



    public AmpSubsystem(int leftPivotID, int rightPivotID) {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                45,
                9,
                5)
                ;

        m_armPositionPIDController = new PIDController(2.5, 0.1, 0);
        m_lowLevelFeedforwardConstants = new FeedforwardConstants(0.122,0,0.1,0.065);
        m_armVelocityPIDConstants = new PIDConstants(0.0001, 0, 0);


        SmartDashboard.putData("Arm PID Position Controller", m_armPositionPIDController);



        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                //TODO tune kV
                ampAngleServoLeft = ServoFactory.neoAngleServo(
                        m_name + "/Left",
                        leftPivotID,
                        MotorPhase.FORWARD,
                        kCurrentLimit,
                        m_params,
                        m_armPositionPIDController, //2.5 0.1
                        m_lowLevelFeedforwardConstants,
                        m_armVelocityPIDConstants); //Where did this come from?

                ampAngleServoRight = ServoFactory.neoAngleServo(
                        m_name + "/Right",
                        rightPivotID,
                        MotorPhase.REVERSE,
                        kCurrentLimit,
                        m_params,
                        m_armPositionPIDController,
                        m_lowLevelFeedforwardConstants,
                        m_armVelocityPIDConstants);
                break;
            case BLANK:
            default:
                ampAngleServoLeft = ServoFactory.simulatedAngleServo(
                        m_name + "/Left",
                        m_params,
                        new PIDController(1, 0, 0));
                ampAngleServoRight = ServoFactory.simulatedAngleServo(
                        m_name + "/Right",
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
        System.out.println("AHHHHHHHHHHHH");
        ampAngleServoRight.setPosition(value);
        ampAngleServoLeft.setPosition(value);
    }

    public void stop() {
        ampAngleServoRight.stop();
        ampAngleServoLeft.stop();
    }

    @Override
    public double getPositionRad() {
        return (ampAngleServoRight.getPosition() + ampAngleServoLeft.getPosition()) / 2;
    }

    public boolean inPosition() {
        // TODO get real values here
        return getPositionRad() < 0.75 * Math.PI && getPositionRad() > .5 * Math.PI;
    }

    @Override
    public void periodic() {
        ampAngleServoRight.periodic();
        ampAngleServoLeft.periodic();
        m_viz.periodic();
    }
}
