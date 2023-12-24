package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.VelocityLimitRegionConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is from 254 2023.
 */
public class TrajectoryGenerator100 {
    private static final double kMaxAccel = 2.54;
    public static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot

    private final TrajectoryPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public TrajectoryGenerator100(TrajectoryPlanner motion_planner) {
        mMotionPlanner = motion_planner;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            mTrajectorySet = new TrajectorySet();
        }
    }

    public void forceRegenerateTrajectories() {
        mTrajectorySet = new TrajectorySet();
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double max_vel,
            double max_accel) {
        return mMotionPlanner.generateTrajectory(
                reversed,
                waypoints,
                headings,
                constraints,
                max_vel,
                max_accel);
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_accel) {
        return mMotionPlanner.generateTrajectory(
                reversed,
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                max_vel,
                max_accel);
    }

    public class TrajectorySet {
        public final Trajectory100 testTrajectory;
        public final Trajectory100 testTrajectory2;
        public final Trajectory100 coopLinkStartToCoopFirstPickup;
        public final Trajectory100 coopFirstPickupToCoopSecondScore;
        public final Trajectory100 coopFirstScoreToDock;
        public final Trajectory100 coopSecondScoreToDock;
        public final Trajectory100 redNCPCubeScoringPositionToFarSideDockWithPickup;
        public final Trajectory100 redNCPCubeScoringPositionToThirdCube;
        public final Trajectory100 redCPLeftScoringPositionToCPFirstPickup;
        public final Trajectory100 CPFirstPickupToFarSideDock;
        public final Trajectory100 ncpScoringToCubePickup;
        public final Trajectory100 ncpThirdScoreToBackoff;

        public final Trajectory100 cpFirstScoreToOutsidePickup;
        public final Trajectory100 cpFirstScoreToInsidePickup;
        public final Trajectory100 cpOutsidePickupToSecondAlign;
        public final Trajectory100 cpInsidePickupToSecondAlign;
        public final Trajectory100 cpSecondScoreToInsidePickup;
        public final Trajectory100 cpSecondScoreToOutsidePickup;
        public final Trajectory100 cpInsidePickupToDock;
        public final Trajectory100 cpOutsidePickupToDock;
        public final Trajectory100 cpInsidePickupToThirdScore;
        public final Trajectory100 cpOutsidePickupToThirdScore;
        public final Trajectory100 cpThirdScoreToBackoff;

        public final VelocityLimitRegionConstraint center_outbound_charging_station_constraint;
        public final VelocityLimitRegionConstraint center_inbound_charging_station_constraint;

        private List<Trajectory100> allTrajectories;

        public List<Trajectory100> getAllTrajectories() {
            return allTrajectories;
        }

        private TrajectorySet() {

            center_outbound_charging_station_constraint = new VelocityLimitRegionConstraint(
                    new Translation2d(1.0, -1.27),
                    new Translation2d(3.0, 1.27),
                    1.25);

            center_inbound_charging_station_constraint = new VelocityLimitRegionConstraint(
                    new Translation2d(0.0, -1.27),
                    new Translation2d(4.5, 1.27),
                    1.25);

            allTrajectories = new ArrayList<>();
            testTrajectory = getTestTrajectory();
            allTrajectories.add(testTrajectory);
            testTrajectory2 = getTestTrajectory2();
            allTrajectories.add(testTrajectory2);
            coopLinkStartToCoopFirstPickup = getCoopLinkStartToCoopFirstPickup();
            allTrajectories.add(coopLinkStartToCoopFirstPickup);
            coopFirstPickupToCoopSecondScore = getCoopFirstPickupToCoopSecondScore();
            allTrajectories.add(coopFirstPickupToCoopSecondScore);
            coopFirstScoreToDock = getCoopFirstScoreToDock();
            allTrajectories.add(coopFirstScoreToDock);
            coopSecondScoreToDock = getCoopSecondScoreToDock();
            allTrajectories.add(coopSecondScoreToDock);
            redNCPCubeScoringPositionToFarSideDockWithPickup = getRedNCPCubeScoringPositionToFarSideDockWithPickup();
            allTrajectories.add(redNCPCubeScoringPositionToFarSideDockWithPickup);
            redNCPCubeScoringPositionToThirdCube = getRedNCPCubeScoringPositionToThirdCube();
            allTrajectories.add(redNCPCubeScoringPositionToThirdCube);
            redCPLeftScoringPositionToCPFirstPickup = getRedCPLeftScoringPositionToCPFirstPickup();
            allTrajectories.add(redCPLeftScoringPositionToCPFirstPickup);
            CPFirstPickupToFarSideDock = getCPFirstPickupToFarSideDock();
            allTrajectories.add(CPFirstPickupToFarSideDock);
            ncpScoringToCubePickup = getNCPScoringtoCubePickup();
            allTrajectories.add(ncpScoringToCubePickup);
            cpFirstScoreToOutsidePickup = getCPFirstScoreToOutsidePickup();
            allTrajectories.add(cpFirstScoreToOutsidePickup);
            cpFirstScoreToInsidePickup = getCPFirstScoreToInsidePickup();
            allTrajectories.add(cpFirstScoreToInsidePickup);
            cpOutsidePickupToSecondAlign = getCPOutsidePickupToSecondAlign();
            allTrajectories.add(cpOutsidePickupToSecondAlign);
            cpInsidePickupToSecondAlign = getCPInsidePickupToSecondAlign();
            allTrajectories.add(cpInsidePickupToSecondAlign);
            cpSecondScoreToInsidePickup = getCPSecondScoreToInsidePickup();
            allTrajectories.add(cpSecondScoreToInsidePickup);
            cpSecondScoreToOutsidePickup = getCPSecondScoreToOutsidePickup();
            allTrajectories.add(cpSecondScoreToOutsidePickup);
            cpInsidePickupToDock = getCPInsidePickupToDock();
            allTrajectories.add(cpInsidePickupToDock);
            cpOutsidePickupToDock = getCPOutsidePickupToDock();
            allTrajectories.add(cpOutsidePickupToDock);
            cpInsidePickupToThirdScore = getCPInsidePickupToThirdScore();
            allTrajectories.add(cpInsidePickupToThirdScore);
            cpOutsidePickupToThirdScore = getCPOutsidePickupToThirdScore();
            allTrajectories.add(cpOutsidePickupToThirdScore);
            cpThirdScoreToBackoff = getCPThirdScoreToBackoff();
            allTrajectories.add(cpThirdScoreToBackoff);
            ncpThirdScoreToBackoff = getNcpThirdScoreToBackoff();
            allTrajectories.add(ncpThirdScoreToBackoff);
        }

        private Trajectory100 getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(0));
            return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
        }

        private Trajectory100 getTestTrajectory2() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.3, 1.0);
        }

        private Trajectory100 getCPFirstScoreToOutsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.9652, -0.125, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.85, -0.125, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.1832, -0.4, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(5.2832, -0.45, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory100 getCPFirstScoreToInsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.016, -0.362, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(2.8447999999999998, -0.362, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(5.2, -1.5, Rotation2d.fromDegrees(-20.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(-25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.57);
        }

        private Trajectory100 getCPOutsidePickupToSecondAlign() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.2832, -0.45, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(2.845, -0.5, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-179));
            waypoints.add(new Pose2d(1.2176, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.5016, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.1, -0.62, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.57);
        }

        private Trajectory100 getCPInsidePickupToSecondAlign() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(180 - 25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(2.81, -0.762, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(1.9176, -0.762, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.8176, -0.762, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.1, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));
            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory100 getCPSecondScoreToInsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.6, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(1.016, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(2.8447999999999998, -0.3, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(5.1, -1.6, Rotation2d.fromDegrees(-20.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(5.5, -1.9, Rotation2d.fromDegrees(-25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory100 getCPSecondScoreToOutsidePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.8, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            waypoints.add(new Pose2d(0.9652, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.8447999999999998, -0.25, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.1832, -0.508, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(-0.01));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory100 getCPInsidePickupToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.5, -1.8, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(4.50, -2.25, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(0.0));

            waypoints.add(new Pose2d(3.0, -2.25, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 0.7, 1.75);
        }

        private Trajectory100 getCPOutsidePickupToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(-0.01));

            waypoints.add(new Pose2d(4.50, -2.25, Rotation2d.fromDegrees(-135)));
            headings.add(Rotation2d.fromDegrees(0.0));

            waypoints.add(new Pose2d(3.0, -2.25, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 0.8, 1.75);
        }

        private Trajectory100 getCPInsidePickupToThirdScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.5, -1.9, Rotation2d.fromDegrees(180 - 25.158531783)));
            headings.add(Rotation2d.fromDegrees(-60));

            waypoints.add(new Pose2d(3.01, -0.75, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(1.9176, -0.75, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.8176, -0.75, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            waypoints.add(new Pose2d(0.1, -0.72, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.57);
        }

        private Trajectory100 getCPOutsidePickupToThirdScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(5.2832, -0.508, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(-0.01));
            waypoints.add(new Pose2d(2.8447999999999998, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-179));
            waypoints.add(new Pose2d(1.2176, -0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.5016, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180));
            waypoints.add(new Pose2d(0.2, -0.8, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(-180.0));

            return generate(waypoints, headings, List.of(), false, 0.75, 1.7);
        }

        private Trajectory100 getCPThirdScoreToBackoff() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.1, -0.72, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180));
            waypoints.add(new Pose2d(1.0, -0.50, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180));
            waypoints.add(new Pose2d(2.59, -0.5, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(5.18, -0.6, Rotation2d.fromDegrees(45.0)));
            headings.add(Rotation2d.fromDegrees(180.0));

            return generate(waypoints, headings, List.of(), false, 1.0, 2.0);
        }

        private Trajectory100 generate(
                List<Pose2d> waypoints,
                List<Rotation2d> headings,
                List<TimingConstraint> constraints,
                boolean reversed,
                double percentSpeed,
                double percentAccel) {
            handleAllianceFlip(waypoints, headings);

            return generateTrajectory(
                    reversed,
                    waypoints,
                    headings,
                    constraints,
                    percentSpeed * kMaxVelocityMetersPerSecond,
                    percentAccel * kMaxAccel);
        }

        private void handleAllianceFlip(List<Pose2d> waypoints, List<Rotation2d> headings) {
            SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue) {
                for (int i = 0; i < waypoints.size(); i++) {
                    waypoints.set(i,
                            new Pose2d(
                                    new Translation2d(waypoints.get(i).getTranslation().getX(),
                                            -waypoints.get(i).getTranslation().getY()),
                                    waypoints.get(i).getRotation().unaryMinus()));
                }
                for (int i = 0; i < headings.size(); i++) {
                    headings.set(i, headings.get(i).unaryMinus());
                }
            }
        }

        private Trajectory100 getCoopLinkStartToCoopFirstPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.75, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(6.0, 0.20, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8,
                    1.375);
        }

        private Trajectory100 getCoopFirstPickupToCoopSecondScore() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(6.0, 0.20, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            waypoints.add(new Pose2d(4.5, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.75, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.2, -0.40, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_inbound_charging_station_constraint), false, 0.8,
                    1.375);
        }

        public Trajectory100 getNcpThirdScoreToBackoff() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();

            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.50, 0.05, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(5.87, -0.47, Rotation2d.fromDegrees(-10.0)));
            headings.add(Rotation2d.fromDegrees(0.0));

            return generate(waypoints, headings, List.of(), false, 1.0, 2.0);
        }

        private Trajectory100 getCoopFirstScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8, 1.0);
        }

        private Trajectory100 getCoopSecondScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), false, 0.8, 1.0);
        }

        private Trajectory100 getNCPScoringtoCubePickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.127, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, -0.4588, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.534799999999999, -0.43179999999999996, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.3038, 0.5111999999999999, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.302, 0.308, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.2191999999999998, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory100 getRedNCPCubeScoringPositionToFarSideDockWithPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 0.8, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 1.6, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.5, 2.0, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory100 getRedNCPCubeScoringPositionToThirdCube() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.143, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(3.7592, 0.327, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 0.8, Rotation2d.fromDegrees(90.0)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(5.5, 1.6, Rotation2d.fromDegrees(135)));
            headings.add(Rotation2d.fromDegrees(134.99999999999994));
            waypoints.add(new Pose2d(3.302, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.2191999999999998, 0.408, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.05, 0.55, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.7, 1.0);
        }

        private Trajectory100 getRedCPLeftScoringPositionToCPFirstPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.3495, -0.1016, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(90.0));
            waypoints.add(new Pose2d(5.334, -0.254, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
        }

        private Trajectory100 getCPFirstPickupToFarSideDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(5.334, -0.254, Rotation2d.fromDegrees(180.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            waypoints.add(new Pose2d(4.191, -2.032, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(0.0));
            return generate(waypoints, headings, List.of(), false, 0.5, 1.0);
        }
    }
}