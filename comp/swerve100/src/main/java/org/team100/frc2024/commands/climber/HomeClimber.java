package org.team100.frc2024.commands.climber;

import org.team100.frc2024.motion.climber.ClimberSubsystem;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The zeroing process has four states:
 * 
 * <pre>
 * initial: set the effort to low and start the start timer
 * 
 * starting state: run at the target speed
 * 
 * transition: when the start timer runs out:
 * 
 * running state: run at the target speed
 * 
 * transition: when the actual speed is below the threshold, start the stop timer
 * 
 * stopping state: run at the target speed
 * 
 * transition: when the stop timer runs out, zero the mechanism
 * 
 * done state: set speed to zero
 * </pre>
 */
public class HomeClimber extends Command {
    private static final double kTargetSpeedM_S = 0.02;
    private static final double kThresholdSpeedM_S = 0.002;
    private static final double kStartTimeS = 0.5;
    private static final double kHomeTimeS = 0.5;
    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final Timer m_leftStartTimer;
    private final Timer m_leftDoneTimer;
    private final Timer m_rightStartTimer;
    private final Timer m_rightDoneTimer;

    public HomeClimber(
            SupplierLogger logger,
            ClimberSubsystem climber) {
        m_logger = logger;
        m_climber = climber;
        m_leftStartTimer = new Timer();
        m_leftDoneTimer = new Timer();
        m_rightStartTimer = new Timer();
        m_rightDoneTimer = new Timer();
    }

    @Override
    public void initialize() {
        m_leftStartTimer.restart();
        m_rightStartTimer.restart();
    }

    @Override
    public void execute() {
        m_climber.setLeftVelocityM_S(kTargetSpeedM_S);
        m_climber.setRightVelocityM_S(kTargetSpeedM_S);
        if (  m_climber.getVelocityM_S() < kThresholdSpeedM_S)
    }

    private static void oneSide() {

    }

    @Override
    public boolean isFinished() {
        return m_leftDoneTimer.hasElapsed(kHomeTimeS)
                && m_rightDoneTimer.hasElapsed(kHomeTimeS)
                && m_climber.getVelocityM_S() < kThresholdSpeedM_S;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            m_climber.zeroClimbers();
    }

}
