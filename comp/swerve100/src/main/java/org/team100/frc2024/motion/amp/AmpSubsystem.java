package org.team100.frc2024.motion.amp;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Positioning;
import org.team100.lib.motion.simple.SimpleVisualization;
import org.team100.lib.units.Angle100;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpSubsystem extends SubsystemBase implements Positioning {
    private final String m_name;
    private final SysParam m_params;
    private final PositionServo<Angle100> ampAngleMotorLeft;
    private final PositionServo<Angle100> ampAngleMotorRight;
    private final SimpleVisualization m_viz;

    public AmpSubsystem(int leftPivotID, int rightPivotID) {
        m_name = Names.name(this);
        m_params = SysParam.neoPositionServoSystem(15, 8, 20);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                ampAngleMotorLeft = ServoFactory.neoAngleServo(
                        m_name + "/Left",
                        leftPivotID,
                        false,
                        m_params,
                        new PIDController(1, 0, 0));

                ampAngleMotorRight = ServoFactory.neoAngleServo(
                        m_name + "/Right",
                        rightPivotID,
                        false,
                        m_params,
                        new PIDController(1, 0, 0));
                break;
            case BLANK:
            default:
                ampAngleMotorLeft = ServoFactory.simulatedAngleServo(
                        m_name + "/Left",
                        m_params,
                        new PIDController(1, 0, 0));
                ampAngleMotorRight = ServoFactory.simulatedAngleServo(
                        m_name + "/Right",
                        m_params,
                        new PIDController(1, 0, 0));
        }
        m_viz = new SimpleVisualization(m_name, this);
    }

    public void setAmpPosition(double value, double value2) {
        ampAngleMotorRight.setPosition(value);
        ampAngleMotorLeft.setPosition(value2);

    }

    public void setAmpVelocity(double value) {
        ampAngleMotorRight.setVelocity(value);
        ampAngleMotorLeft.setVelocity(value);
    }

    public double getLeftAmpPosition() {
        return ampAngleMotorLeft.getPosition();
    }

    public double getRightAmpPosition() {
        return ampAngleMotorRight.getPosition();
    }

    @Override
    public double getPosition() {
        return (getRightAmpPosition() + getLeftAmpPosition()) / 2;
    }

    @Override
    public void periodic() {
        ampAngleMotorRight.periodic();
        ampAngleMotorLeft.periodic();
        m_viz.periodic();
    }
}
