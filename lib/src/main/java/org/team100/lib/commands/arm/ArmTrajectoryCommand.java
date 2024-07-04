package org.team100.lib.commands.arm;

import java.util.Optional;

import org.team100.lib.commands.Command100;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.motion.arm.ArmTrajectories;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;

/**
 * Trajectory follower for the two-jointed arm.
 * 
 * Sample a straight-line trajectory in cartesian coordinates, transform
 * position, velocity and acceleration to joint space, and follow with velocity
 * feedforward and positional feedback, both using constant parameters (i.e. no
 * gravity feedforward, no inertia-dependent feedback).
 */
public class ArmTrajectoryCommand extends Command100 {
    private static final double kTolerance = 0.02;
    private static final TrajectoryConfig kConf = new TrajectoryConfig(1, 1);
    private static final double kA = 0.2;
    private static final double kRotsPerSecToVoltsPerSec = 4;

    private final ArmSubsystem m_armSubsystem;
    private final ArmKinematics m_armKinematicsM;
    private final Translation2d m_goal;

    private final ArmAngles m_goalAngles;
    private final Timer m_timer;

    private final PIDController m_lowerPosController;
    private final PIDController m_upperPosController;
    private final PIDController m_lowerVelController;
    private final PIDController m_upperVelController;

    private final ArmTrajectories m_trajectories;

    private Trajectory m_trajectory;

    public ArmTrajectoryCommand(
            Logger parent,
            ArmSubsystem armSubSystem,
            ArmKinematics armKinematicsM,
            Translation2d goal) {
        super(parent);
        m_armSubsystem = armSubSystem;
        m_armKinematicsM = armKinematicsM;
        m_goal = goal;

        m_goalAngles = m_armKinematicsM.inverse(m_goal);
        m_timer = new Timer();

        m_lowerPosController = pController(2, 0.1);
        m_upperPosController = pController(2, 0.05);
        m_lowerVelController = controller(0.1, 0);
        m_upperVelController = controller(0.1, 0);

        m_trajectories = new ArmTrajectories(kConf);

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize100() {
        m_timer.restart();
        Optional<ArmAngles> position = m_armSubsystem.getPosition();
        if (position.isEmpty())
            return;
        m_trajectory = m_trajectories.makeTrajectory(
                m_armKinematicsM.forward(position.get()), m_goal);
    }

    @Override
    public void execute100(double dt) {
        if (m_trajectory == null)
            return;
        if (m_goalAngles == null)
            return;

        State desiredState = getDesiredState();

        Optional<ArmAngles> measurement = m_armSubsystem.getPosition();
        if (measurement.isEmpty())
            return;
        Optional<ArmAngles> velocityMeasurement = m_armSubsystem.getVelocity();
        if (velocityMeasurement.isEmpty())
            return;

        // position reference
        ArmAngles r = getThetaPosReference(desiredState);

        // position feedback
        double u1_pos = m_lowerPosController.calculate(measurement.get().th1, r.th1);
        double u2_pos = m_upperPosController.calculate(measurement.get().th2, r.th2);

        // velocity reference
        ArmAngles rdot = getThetaVelReference(desiredState, r);

        // feedforward
        double ff2 = rdot.th2 / (Math.PI * 2) * kRotsPerSecToVoltsPerSec;
        double ff1 = rdot.th1 / (Math.PI * 2) * kRotsPerSecToVoltsPerSec;

        // velocity feedback
        double u1_vel = m_lowerVelController.calculate(velocityMeasurement.get().th1, rdot.th1);
        double u2_vel = m_upperVelController.calculate(velocityMeasurement.get().th2, rdot.th2);

        double u1 = ff1 + u1_pos + u1_vel;
        double u2 = ff2 + u2_pos + u2_vel;

        m_armSubsystem.set(u1, u2);

        m_logger.logDouble(Level.TRACE, "Lower FF ", () -> ff1);
        m_logger.logDouble(Level.TRACE, "Lower Controller Output: ", () -> u1_pos);
        m_logger.logDouble(Level.TRACE, "Upper FF ", () -> ff2);
        m_logger.logDouble(Level.TRACE, "Upper Controller Output: ", () -> u2_pos);
        m_logger.logDouble(Level.TRACE, "Lower Ref: ", () -> r.th1);
        m_logger.logDouble(Level.TRACE, "Upper Ref: ", () -> r.th2);
        m_logger.logDouble(Level.TRACE, "Output Upper: ", () -> u1);
        m_logger.logDouble(Level.TRACE, "Output Lower: ", () -> u2);
    }

    private State getDesiredState() {
        double curTime = m_timer.get();
        State state = m_trajectory.sample(curTime);
        // the last state in the trajectory includes whatever the terminal acceleration
        // was, so if you keep sampling past the end, you'll be trying to accelerate
        // away from the endpoint, even though the desired velocity is zero. :-(
        // so we just fix it here:
        if (curTime > m_trajectory.getTotalTimeSeconds())
            state.accelerationMetersPerSecondSq = 0;
        return state;
    }

    ArmAngles getThetaPosReference(State desiredState) {
        Translation2d XYPosReference = desiredState.poseMeters.getTranslation();
        return m_armKinematicsM.inverse(XYPosReference);
    }

    ArmAngles getThetaVelReference(
            State desiredState,
            ArmAngles thetaPosReference) {
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        // accounting for acceleration along the path.
        // in general, we should also account for acceleration across the path, i.e.
        // curvature, but in this case we know the path is a straight line,
        // so just boost the desired velocity a little.
        desiredVecloity += kA * desiredState.accelerationMetersPerSecondSq;
        Rotation2d theta = desiredState.poseMeters.getRotation();
        double desiredXVel = desiredVecloity * theta.getCos();
        double desiredYVel = desiredVecloity * theta.getSin();
        Translation2d XYVelReference = new Translation2d(desiredXVel, desiredYVel);
        return m_armKinematicsM.inverseVel(thetaPosReference, XYVelReference);

    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        if (m_goalAngles == null)
            return true;

        return m_timer.get() > m_trajectory.getTotalTimeSeconds()
                && m_lowerPosController.atSetpoint()
                && m_upperPosController.atSetpoint()
                && m_lowerVelController.atSetpoint()
                && m_upperVelController.atSetpoint();
    }

    @Override
    public void end100(boolean interrupted) {
        m_armSubsystem.set(0, 0);
        m_trajectory = null;
    }

    ////////////////////////////////

    private static PIDController controller(double p, double d) {
        PIDController c = new PIDController(p, 0, d);
        c.setTolerance(kTolerance);
        return c;
    }

    /** Position controller is continuous. */
    private static PIDController pController(double p, double d) {
        PIDController c = controller(p, d);
        c.enableContinuousInput(-Math.PI, Math.PI);
        return c;
    }
}