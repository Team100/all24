package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.VelocityLimitRegionConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is from 254 2023.
 */
public class TrajectoryGenerator100 {
    private static final double kMaxAccel = 2.54;
    public static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot

    private final TrajectorySet mTrajectorySet;

    public TrajectoryGenerator100() {
        mTrajectorySet = new TrajectorySet();
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
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
        // public final Trajectory100 CPFirstPickupToFarSideDock;
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

        private Map<String, Trajectory100> allTrajectories;

        public Map<String, Trajectory100> getAllTrajectories() {
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

            allTrajectories = new HashMap<>();
            testTrajectory = getTestTrajectory();
            allTrajectories.put("testTrajectory", testTrajectory);
            testTrajectory2 = getTestTrajectory2();
            allTrajectories.put("testTrajectory2", testTrajectory2);
            coopLinkStartToCoopFirstPickup = getCoopLinkStartToCoopFirstPickup();
            allTrajectories.put("coopLinkStartToCoopFirstPickup", coopLinkStartToCoopFirstPickup);
            coopFirstPickupToCoopSecondScore = getCoopFirstPickupToCoopSecondScore();
            allTrajectories.put("coopFirstPickupToCoopSecondScore", coopFirstPickupToCoopSecondScore);
            coopFirstScoreToDock = getCoopFirstScoreToDock();
            allTrajectories.put("coopFirstScoreToDock", coopFirstScoreToDock);
            coopSecondScoreToDock = getCoopSecondScoreToDock();
            allTrajectories.put("coopSecondScoreToDock", coopSecondScoreToDock);
            redNCPCubeScoringPositionToFarSideDockWithPickup = getRedNCPCubeScoringPositionToFarSideDockWithPickup();
            allTrajectories.put("redNCPCubeScoringPositionToFarSideDockWithPickup", redNCPCubeScoringPositionToFarSideDockWithPickup);
            redNCPCubeScoringPositionToThirdCube = getRedNCPCubeScoringPositionToThirdCube();
            allTrajectories.put("redNCPCubeScoringPositionToThirdCube", redNCPCubeScoringPositionToThirdCube);
            redCPLeftScoringPositionToCPFirstPickup = getRedCPLeftScoringPositionToCPFirstPickup();
            allTrajectories.put("redCPLeftScoringPositionToCPFirstPickup", redCPLeftScoringPositionToCPFirstPickup);
            // CPFirstPickupToFarSideDock = getCPFirstPickupToFarSideDock();
            // allTrajectories.put("CPFirstPickupToFarSideDock", CPFirstPickupToFarSideDock);
            ncpScoringToCubePickup = getNCPScoringtoCubePickup();
            allTrajectories.put("ncpScoringToCubePickup", ncpScoringToCubePickup);
            cpFirstScoreToOutsidePickup = getCPFirstScoreToOutsidePickup();
            allTrajectories.put("cpFirstScoreToOutsidePickup", cpFirstScoreToOutsidePickup);
            cpFirstScoreToInsidePickup = getCPFirstScoreToInsidePickup();
            allTrajectories.put("cpFirstScoreToInsidePickup", cpFirstScoreToInsidePickup);
            cpOutsidePickupToSecondAlign = getCPOutsidePickupToSecondAlign();
            allTrajectories.put("cpOutsidePickupToSecondAlign", cpOutsidePickupToSecondAlign);
            cpInsidePickupToSecondAlign = getCPInsidePickupToSecondAlign();
            allTrajectories.put("cpInsidePickupToSecondAlign", cpInsidePickupToSecondAlign);
            cpSecondScoreToInsidePickup = getCPSecondScoreToInsidePickup();
            allTrajectories.put("cpSecondScoreToInsidePickup", cpSecondScoreToInsidePickup);
            cpSecondScoreToOutsidePickup = getCPSecondScoreToOutsidePickup();
            allTrajectories.put("cpSecondScoreToOutsidePickup", cpSecondScoreToOutsidePickup);
            cpInsidePickupToDock = getCPInsidePickupToDock();
            allTrajectories.put("cpInsidePickupToDock", cpInsidePickupToDock);
            cpOutsidePickupToDock = getCPOutsidePickupToDock();
            allTrajectories.put("cpOutsidePickupToDock", cpOutsidePickupToDock);
            cpInsidePickupToThirdScore = getCPInsidePickupToThirdScore();
            allTrajectories.put("cpInsidePickupToThirdScore", cpInsidePickupToThirdScore);
            cpOutsidePickupToThirdScore = getCPOutsidePickupToThirdScore();
            allTrajectories.put("cpOutsidePickupToThirdScore", cpOutsidePickupToThirdScore);
            cpThirdScoreToBackoff = getCPThirdScoreToBackoff();
            allTrajectories.put("cpThirdScoreToBackoff", cpThirdScoreToBackoff);
            ncpThirdScoreToBackoff = getNcpThirdScoreToBackoff();
            allTrajectories.put("ncpThirdScoreToBackoff", ncpThirdScoreToBackoff);
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
            return generate(waypoints, headings, List.of(), 0.8, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.3, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.75, 1.57);
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
            return generate(waypoints, headings, List.of(), 0.7, 1.57);
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
            return generate(waypoints, headings, List.of(), 0.7, 1.57);
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
            return generate(waypoints, headings, List.of(), 0.75, 1.57);
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

            return generate(waypoints, headings, List.of(), 0.75, 1.57);
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

            return generate(waypoints, headings, List.of(), 0.75, 1.57);
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

            return generate(waypoints, headings, List.of(), 0.7, 1.75);
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

            return generate(waypoints, headings, List.of(), 0.8, 1.75);
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

            return generate(waypoints, headings, List.of(), 0.75, 1.57);
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

            return generate(waypoints, headings, List.of(), 0.75, 1.7);
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

            return generate(waypoints, headings, List.of(), 1.0, 2.0);
        }

        private Trajectory100 generate(
                List<Pose2d> waypoints,
                List<Rotation2d> headings,
                List<TimingConstraint> constraints,
                double percentSpeed,
                double percentAccel) {

            final List<Pose2d> waypoints1 = waypoints;
            final List<Rotation2d> headings1 = headings;
            final List<TimingConstraint> constraints1 = constraints;
            return TrajectoryPlanner.generateTrajectory(
                    waypoints1,
                    headings1,
                    constraints1,
                    0.0,
                    0.0,
                    percentSpeed * kMaxVelocityMetersPerSecond,
                    percentAccel * kMaxAccel);
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
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), 0.8,
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
            return generate(waypoints, headings, List.of(center_inbound_charging_station_constraint), 0.8,
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

            return generate(waypoints, headings, List.of(), 1.0, 2.0);
        }

        private Trajectory100 getCoopFirstScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, 0.00, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), 0.8, 1.0);
        }

        private Trajectory100 getCoopSecondScoreToDock() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.2, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.0, -0.40, Rotation2d.fromDegrees(0.0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(center_outbound_charging_station_constraint), 0.8, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.7, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.7, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.7, 1.0);
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
            return generate(waypoints, headings, List.of(), 0.5, 1.0);
        }

        // private Trajectory100 getCPFirstPickupToFarSideDock() {
        // List<Pose2d> waypoints = new ArrayList<>();
        // List<Rotation2d> headings = new ArrayList<>();
        // waypoints.add(new Pose2d(5.334, -0.254, Rotation2d.fromDegrees(180.0)));
        // headings.add(Rotation2d.fromDegrees(0.0));
        // waypoints.add(new Pose2d(4.191, -2.032, Rotation2d.fromDegrees(0.0)));
        // headings.add(Rotation2d.fromDegrees(0.0));
        // return generate(waypoints, headings, List.of(), 0.5, 1.0);
        // }
    }
}