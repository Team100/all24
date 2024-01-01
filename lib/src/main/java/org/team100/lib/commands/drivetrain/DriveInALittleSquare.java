package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.Constraints;
import org.team100.lib.profile.State;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Makes a little square, one meter on a side, forever.
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
public class DriveInALittleSquare extends Command100 {
    enum DriveState {
        DRIVING,
        STEERING
    }

    /** square should be 1 m on a side. */
    private static final double kDriveLengthM = 1;
    private static final double kMaxVel = 1;
    private static final double kMaxAccel = 1;

    private static final double kXToleranceRad = 0.02;
    private static final double kVToleranceRad_S = 0.02;

    private final SwerveDriveSubsystem m_swerve;
    private final State start = new State(0, 0);
    private final State goal = new State(kDriveLengthM, 0);

    final TrapezoidProfile100 m_driveProfile;
    /** Current speed setpoint. */
    State speedM_S;
    /** Current swerve steering axis goal. */
    Rotation2d m_goal;
    DriveState m_state;

    public DriveInALittleSquare(SwerveDriveSubsystem swerve) {
        m_swerve = swerve;

        Constraints c = new Constraints(kMaxVel, kMaxAccel);
        m_driveProfile = new TrapezoidProfile100(c, 0.05);
        addRequirements(m_swerve);
    }

    @Override
    public void initialize100() {
        // First get the wheels pointing the right way.
        m_state = DriveState.STEERING;
        m_goal = GeometryUtil.kRotationZero;
        speedM_S = start;
        speedM_S = m_driveProfile.calculate(0, speedM_S, goal);
    }

    @Override
    public void execute100(double dt) {
        switch (m_state) {
            case DRIVING:
                if (MathUtil.isNear(speedM_S.position, goal.position, kXToleranceRad)
                        && MathUtil.isNear(speedM_S.velocity, goal.velocity, kVToleranceRad_S)) {
                    // we were driving, but the timer elapsed, so switch to steering
                    m_state = DriveState.STEERING;
                    m_goal = m_goal.plus(GeometryUtil.kRotation90);
                    speedM_S = start;
                } else {
                    // keep going
                    speedM_S = m_driveProfile.calculate(dt, speedM_S, goal);
                }
                break;
            case STEERING:
                if (Util.all(m_swerve.atGoal())) {
                    // we were steering, but all the setpoints have been reached, so switch to
                    // driving
                    m_state = DriveState.DRIVING;
                    speedM_S = start;
                    speedM_S = m_driveProfile.calculate(dt, speedM_S, goal);
                } else {
                    // wait to reach the setpoint
                }
                break;
        }

        // there are four states here because state is mutable :-(
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(speedM_S.position, m_goal),
                new SwerveModuleState(speedM_S.position, m_goal),
                new SwerveModuleState(speedM_S.position, m_goal),
                new SwerveModuleState(speedM_S.position, m_goal)
        };
        m_swerve.setRawModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }
}
