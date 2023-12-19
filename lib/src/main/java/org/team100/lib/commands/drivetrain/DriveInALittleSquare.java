package org.team100.lib.commands.drivetrain;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Makes a little square.
 * 
 * This is intended for tuning the steering control stack, because at each
 * corner of the square, the steering needs to respond quickly and precisely.
 * 
 * It samples a jerk-limited profile for driving, with feedforward to the
 * velocity servo but no positional feedback.
 * 
 * It sends the steering position servo fixed goals, so the servo profile is
 * used.
 */
public class DriveInALittleSquare extends Command {
    private enum State {
        DRIVING,
        STEERING
    }

    private static final double kDriveLengthM = 1;
    private static final double kMaxVel = 1;
    private static final double kMaxAccel = 1;
    private static final double kMaxJerk = 1;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;

    private final MotionProfile m_driveProfile;
    private Rotation2d m_goal;
    private State m_state;

    public DriveInALittleSquare(SwerveDriveSubsystemInterface swerve) {
        m_swerve = swerve;
        m_timer = new Timer();
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(kDriveLengthM, 0, 0, 0);
        m_driveProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goal,
                kMaxVel,
                kMaxAccel,
                kMaxJerk);
        if (m_swerve.get() != null)
            addRequirements(m_swerve.get());
    }

    @Override
    public void initialize() {
        m_goal = GeometryUtil.kRotationZero;
        m_state = State.DRIVING;
        m_timer.restart();
    }

    @Override
    public void execute() {
        double speedM_S = 0;
        switch (m_state) {
            case DRIVING:
                if (m_timer.hasElapsed(m_driveProfile.duration())) {
                    // we were driving, but the timer elapsed, so switch to steering
                    m_state = State.STEERING;
                    m_goal = m_goal.plus(GeometryUtil.kRotation90);
                    speedM_S = 0;
                } else {
                    // keep going
                    speedM_S = m_driveProfile.get(m_timer.get()).getV();
                }
                break;
            case STEERING:
                if (all(m_swerve.atGoal())) {
                    // we were steering, but all the setpoints have been reached, so switch to
                    // driving
                    m_state = State.DRIVING;
                    m_timer.restart();
                    speedM_S = m_driveProfile.get(m_timer.get()).getV();
                } else {
                    // wait to reach the setpoint
                }
                break;
        }

        // there are four states here because state is mutable :-(
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(speedM_S, m_goal),
                new SwerveModuleState(speedM_S, m_goal),
                new SwerveModuleState(speedM_S, m_goal),
                new SwerveModuleState(speedM_S, m_goal)
        };
        m_swerve.setRawModuleStates(states);

    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    private boolean all(boolean[] x) {
        for (boolean b : x) {
            if (!b)
                return false;
        }
        return true;
    }
}
