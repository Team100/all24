package org.team100.frc2024.motion.amp;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.AngularVisualization;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
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

    public AmpSubsystem(int leftPivotID, int rightPivotID) {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(
                15,
                8,
                20);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                ampAngleServoLeft = ServoFactory.neoAngleServo(
                        m_name + "/Left",
                        leftPivotID,
                        true,
                        kCurrentLimit,
                        m_params,
                        new PIDController(1, 0, 0));

                ampAngleServoRight = ServoFactory.neoAngleServo(
                        m_name + "/Right",
                        rightPivotID,
                        false,
                        kCurrentLimit,
                        m_params,
                        new PIDController(1, 0, 0));
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
        if (getPositionRad() < 0.75 * Math.PI && getPositionRad() > .5 * Math.PI) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        ampAngleServoRight.periodic();
        ampAngleServoLeft.periodic();
        m_viz.periodic();
    }
}
