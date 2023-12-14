package org.team100.lib.commands.arm;

import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.motion.arm.ArmTrajectories;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Trajectory follower for the two-jointed arm.
 * 
 * Sample a straight-line trajectory in cartesian coordinates, transform
 * position, velocity and acceleration to joint space, and follow with velocity
 * feedforward and positional feedback, both using constant parameters (i.e. no
 * gravity feedforward, no inertia-dependent feedback).
 */
public class ArmTrajectoryCommand extends Command {
    private static final double kLowerP = 2;
    private static final double kLowerI = 0;
    private static final double kLowerD = 0.1;
    private static final double kUpperP = 2;
    private static final double kUpperI = 0;
    private static final double kUpperD = 0.05;
    private static final double kTolerance = 0.02;
    private static final TrajectoryConfig kConf = new TrajectoryConfig(1, 1);
    private static final double kA = 0.2;
    private static final double kRotsPerSecToVoltsPerSec = 4;

    private final Telemetry t = Telemetry.get();

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
            ArmSubsystem armSubSystem,
            ArmKinematics armKinematicsM,
            Translation2d goal) {
        m_armSubsystem = armSubSystem;
        m_armKinematicsM = armKinematicsM;
        m_goal = goal;
        m_goalAngles = m_armKinematicsM.inverse(m_goal);
        m_timer = new Timer();

        m_lowerPosController = new PIDController(kLowerP, kLowerI, kLowerD);
        m_upperPosController = new PIDController(kUpperP, kUpperI, kUpperD);
        m_lowerPosController.enableContinuousInput(-Math.PI, Math.PI);
        m_upperPosController.enableContinuousInput(-Math.PI, Math.PI);
        m_lowerVelController = new PIDController(0.1, 0, 0);
        m_upperVelController = new PIDController(0.1, 0, 0);

        m_lowerPosController.setIntegratorRange(1, 1);
        m_upperPosController.setIntegratorRange(1, 1);
        m_lowerPosController.setTolerance(kTolerance);
        m_upperPosController.setTolerance(kTolerance);

        m_trajectories = new ArmTrajectories(kConf);

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_trajectory = m_trajectories.makeTrajectory(
                m_armKinematicsM.forward(m_armSubsystem.getPosition()), m_goal);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;

        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        double desiredXPos = desiredState.poseMeters.getX();
        double desiredYPos = desiredState.poseMeters.getY();
        double desiredVecloity = desiredState.velocityMetersPerSecond;
        double desiredAcceleration = desiredState.accelerationMetersPerSecondSq;
        if (desiredState == m_trajectory.sample(100)) {
            desiredAcceleration = 0;
        }
        double theta = desiredState.poseMeters.getRotation().getRadians();
        double desiredXVel = desiredVecloity * Math.cos(theta);
        // TODO: i think Math.sin is what is needed below.
        double desiredYVel = desiredVecloity * Math.cos(Math.PI / 2 - theta);
        double desiredXAccel = desiredAcceleration * Math.cos(theta);
        double desiredYAccel = desiredAcceleration * Math.cos(Math.PI / 2 - theta);

        Translation2d XYVelReference = new Translation2d(desiredXVel, desiredYVel);
        Translation2d XYAccelReference = new Translation2d(desiredXAccel, desiredYAccel);
        XYVelReference = XYVelReference.plus(XYAccelReference.times(kA));

        Translation2d XYPosReference = new Translation2d(desiredXPos, desiredYPos);
        ArmAngles thetaPosReference = m_armKinematicsM.inverse(XYPosReference);
        ArmAngles thetaVelReference = m_armKinematicsM.inverseVel(thetaPosReference, XYVelReference);

        ArmAngles measurement = m_armSubsystem.getPosition();
        double currentLower = measurement.th1;
        double currentUpper = measurement.th2;

        double lowerPosControllerOutput = m_lowerPosController.calculate(currentLower,
                thetaPosReference.th1);
        double lowerVelControllerOutput = m_lowerVelController.calculate(m_armSubsystem.getVelocity().th1,
                thetaVelReference.th1);
        double lowerFeedForward = thetaVelReference.th1 / (Math.PI * 2) * kRotsPerSecToVoltsPerSec;
        double u1 = lowerFeedForward + lowerPosControllerOutput + lowerVelControllerOutput;

        double upperPosControllerOutput = m_upperPosController.calculate(currentUpper,
                thetaPosReference.th2);
        double upperVelControllerOutput = m_upperVelController.calculate(m_armSubsystem.getVelocity().th2,
                thetaVelReference.th2);
        double upperFeedForward = thetaVelReference.th2 / (Math.PI * 2) * kRotsPerSecToVoltsPerSec;
        double u2 = upperFeedForward + upperPosControllerOutput + upperVelControllerOutput;

        m_armSubsystem.set(u1, u2);

        t.log(Level.DEBUG, "/arm_trajectory/Lower FF ", lowerFeedForward);
        t.log(Level.DEBUG, "/arm_trajectory/Lower Controller Output: ", lowerPosControllerOutput);
        t.log(Level.DEBUG, "/arm_trajectory/Upper FF ", upperFeedForward);
        t.log(Level.DEBUG, "/arm_trajectory/Upper Controller Output: ", upperPosControllerOutput);
        t.log(Level.DEBUG, "/arm_trajectory/Lower Ref: ", thetaPosReference.th1);
        t.log(Level.DEBUG, "/arm_trajectory/Upper Ref: ", thetaPosReference.th2);
        t.log(Level.DEBUG, "/arm_trajectory/Output Upper: ", u1);
        t.log(Level.DEBUG, "/arm_trajectory/Output Lower: ", u2);
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return Math.abs(getTrajectoryError().th1) < kTolerance && Math.abs(getTrajectoryError().th2) < kTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.set(0, 0);
        m_trajectory = null;
    }

    private ArmAngles getTrajectoryError() {
        ArmAngles position = m_armSubsystem.getPosition();
        return new ArmAngles(
                position.th1 - m_goalAngles.th1,
                position.th2 - m_goalAngles.th2);
    }
}