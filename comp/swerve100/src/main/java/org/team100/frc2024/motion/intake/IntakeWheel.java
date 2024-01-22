package org.team100.frc2024.motion.intake;

import org.team100.lib.config.Identity;
import org.team100.lib.config.SysParam;
import org.team100.lib.motion.components.LimitedVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.simple.Speeding;
import org.team100.lib.motion.simple.SpeedingVisualization;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;

public class IntakeWheel extends Intake implements Speeding {
    private final String m_name;
    private final LimitedVelocityServo<Distance100> intakeMotor;
    private final SpeedingVisualization m_viz;

    public IntakeWheel(int wheelID) {
        m_name = Names.name(this);

        SysParam params = SysParam.limitedNeoVelocityServoSystem(
                1.0,
                0.05,
                8.0,
                20.0,
                -20.0);
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
                intakeMotor = ServoFactory.limitedNeoVelocityServo(
                        m_name, wheelID, false, params);
                break;
            case BLANK:
            default:
                intakeMotor = ServoFactory.limitedSimulatedVelocityServo(
                        m_name, params);
        }
        m_viz = new SpeedingVisualization(m_name, this);
    }

    @Override
    public void setIntake(double value) {
        intakeMotor.setVelocity(value);
    }

    @Override
    public void periodic() {
        intakeMotor.periodic();
        m_viz.periodic();
    }

    @Override
    public double getVelocity() {
        return intakeMotor.getVelocity();
    }
}