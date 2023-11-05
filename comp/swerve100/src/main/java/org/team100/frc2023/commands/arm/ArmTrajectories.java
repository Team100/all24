package org.team100.frc2023.commands.arm;

import java.util.List;

import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.motion.arm.ArmAngles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class ArmTrajectories {
    public static class Config {
        // Cone
        public ArmAngles highGoalCone = new ArmAngles(0.494, 1.178);
        public ArmAngles midGoalCone = new ArmAngles(0.138339, 1.609977);
        public ArmAngles lowGoalCone = new ArmAngles(0, 2.21);
        public ArmAngles subCone = new ArmAngles(-0.338940, 1.308745);

        // Cube
        public ArmAngles highGoalCube = new ArmAngles(0.316365, 1.147321);
        public ArmAngles midGoalCube = new ArmAngles(0.089803, 1.681915);
        public ArmAngles lowGoalCube = new ArmAngles(-0.049849, 2.271662);
        public ArmAngles subCube = new ArmAngles(-0.341841, 1.361939);
        public ArmAngles subToCube = new ArmAngles(-0.341841, 1.361939);

        public ArmAngles safeBack = new ArmAngles(-0.55, 1.97);
        public ArmAngles safeGoalCone = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeGoalCube = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeWaypoint = new ArmAngles(-0.394089, 1.226285);
    }

    private final Config m_config = new Config();
    private final TrajectoryConfig trajecConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        trajecConfig = config;
    }

    public Trajectory makeTrajectory(ArmAngles start, ArmPosition goal, boolean cubeMode) {
        if (start == null) // unreachable
            return null;
        switch (goal) {
            case SAFEBACK:
                return twoPoint(start, m_config.safeWaypoint, m_config.safeBack, -180);
            case SAFE:
                if (cubeMode)
                    return twoPoint(start, m_config.safeWaypoint, m_config.safeGoalCube, -180);
                return twoPoint(start, m_config.safeWaypoint, m_config.safeGoalCone, -180);
            case SAFEWAYPOINT:
                return onePoint(start, m_config.safeWaypoint, -180);
            case HIGH:
                if (cubeMode)
                    return onePoint(start, m_config.highGoalCube, 90);
                return onePoint(start, m_config.highGoalCone, 90);
            case MID:
                if (cubeMode)
                    return onePoint(start, m_config.midGoalCube, 90);
                return onePoint(start, m_config.midGoalCone, 90);
            case LOW:
                if (cubeMode)
                    return onePoint(start, m_config.lowGoalCube, 90);
                return onePoint(start, m_config.lowGoalCone, 90);
            case SUB:
                if (cubeMode)
                    return onePoint(start, m_config.subCube, 90);
                return onePoint(start, m_config.subCone, 90);
            case SUBTOCUBE:
                if (cubeMode)
                    return onePoint(start, m_config.subToCube, 90);
                return onePoint(start, m_config.subToCube, 90);
        }

        return null;
    }

    /** from current location to an endpoint */
    private Trajectory onePoint(ArmAngles start, ArmAngles end, double degrees) {
        return withList(start, List.of(), end, degrees);
    }

    /** from current location, through a waypoint, to an endpoint */
    private Trajectory twoPoint(ArmAngles start, ArmAngles mid, ArmAngles end, double degrees) {
        return withList(start, List.of(new Translation2d(mid.th2, mid.th1)), end, degrees);
    }

    private Trajectory withList(ArmAngles start, List<Translation2d> list, ArmAngles end, double degrees) {
        try {
            return TrajectoryGenerator.generateTrajectory(startPose(start, degrees), list, endPose(end, degrees),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            e.printStackTrace();
            return null;
        }
    }

    private Pose2d startPose(ArmAngles start, double degrees) {
        return new Pose2d(start.th2, start.th1, Rotation2d.fromDegrees(degrees));
    }

    private Pose2d endPose(ArmAngles angles, double degrees) {
        return new Pose2d(angles.th2, angles.th1, Rotation2d.fromDegrees(degrees));
    }

}
