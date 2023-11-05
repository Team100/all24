package org.team100.lib.controller;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.trajectory.TrajectoryTimeIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This originated in DriveMotionPlanner, which included several
 * controllers.
 */
public class DrivePursuitController {

    // TODO: move these states to this class.
    public static ChassisSpeeds updatePurePursuit(
          final  Pose2d current_state,
          final double feedforwardOmegaRadiansPerSecond,
          final Pose2d mError,
          final TrajectoryTimeIterator mCurrentTrajectory,
          final TimedPose mSetpoint,
          final Lookahead mSpeedLookahead,
          final boolean mIsReversed,
          final double defaultCook,
          final double mCurrentTrajectoryLength,
          final boolean useDefaultCook,
          final Rotation2d mPrevHeadingError,
          final double mDt) {
            DriveMotionPlanner.t.log("/planner/error", mError);
    
            double lookahead_time = DriveMotionPlanner.kPathLookaheadTime;
            final double kLookaheadSearchDt = 0.01;
    
            TimedPose lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            DriveMotionPlanner.t.log("/planner/lookahead state", lookahead_state);
    
            double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
            double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.velocityM_S());
            //Find the Point on the Trajectory that is Lookahead Distance Away
            while (actual_lookahead_distance < adaptive_lookahead_distance &&
                    mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
                lookahead_time += kLookaheadSearchDt;
                lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
                actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
            }
    
            //If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead distance away
            if (actual_lookahead_distance < adaptive_lookahead_distance) {
                lookahead_state = new TimedPose(
                    new Pose2dWithMotion(
                       GeometryUtil.transformBy(lookahead_state.state()
                        .getPose(), GeometryUtil.fromTranslation(new Translation2d(
                                (mIsReversed ? -1.0 : 1.0) * (DriveMotionPlanner.kPathMinLookaheadDistance -
                                        actual_lookahead_distance), 0.0))), 0.0), lookahead_state.getTimeS()
                        , lookahead_state.velocityM_S(), lookahead_state.acceleration());
            }
            DriveMotionPlanner.t.log("/planner/updated lookahead state", lookahead_state);
    
            //Find the vector between robot's current position and the lookahead state
            Translation2d lookaheadTranslation = lookahead_state.state().getTranslation().minus(current_state.getTranslation());
            DriveMotionPlanner.t.log("/planner/lookahead translation", lookaheadTranslation);
    
            //Set the steering direction as the direction of the vector
            Rotation2d steeringDirection = lookaheadTranslation.getAngle();
    
            //Convert from field-relative steering direction to robot-relative
            steeringDirection = steeringDirection.rotateBy(GeometryUtil.inverse(current_state).getRotation());
    
            //Use the Velocity Feedforward of the Closest Point on the Trajectory
            double normalizedSpeed = Math.abs(mSetpoint.velocityM_S()) / DriveMotionPlanner.kMaxVelocityMetersPerSecond;
    
            //The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot will drive at the defaultCook speed
            if(normalizedSpeed > defaultCook || mSetpoint.getTimeS() > (mCurrentTrajectoryLength / 2.0)){
                /////////////////////////////???########################
                // TODO: put this back.
    //            useDefaultCook = false;
            }
            if(useDefaultCook){
                normalizedSpeed = defaultCook;
            }
    
            //Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
            final Translation2d steeringVector = new Translation2d(steeringDirection.getCos() * normalizedSpeed, steeringDirection.getSin() * normalizedSpeed);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(steeringVector.getX() * DriveMotionPlanner.kMaxVelocityMetersPerSecond, steeringVector.getY() * DriveMotionPlanner.kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);
    
            DriveMotionPlanner.t.log("/planner/pursuit speeds", chassisSpeeds);
    
            //Use the PD-Controller for To Follow the Time-Parametrized Heading
            final double kThetakP = 3.5;
            final double kThetakD = 0.0;
            final double kPositionkP = 2.0;
    
            chassisSpeeds.vxMetersPerSecond =
                    chassisSpeeds.vxMetersPerSecond + kPositionkP * mError.getTranslation().getX();
            chassisSpeeds.vyMetersPerSecond =
                    chassisSpeeds.vyMetersPerSecond + kPositionkP * mError.getTranslation().getY();
            chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + (kThetakP * mError.getRotation().getRadians()) + kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
            return chassisSpeeds;
        }
    
}
