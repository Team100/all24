package frc.robot.arm;

import java.util.List;

import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;

public class ArmTrajectories {
    public static class Config {
        // Cone
        public ArmKinematics kinematics = new ArmKinematics(0.93, .92);
        public Translation2d test = new Translation2d(3, 3);
        public Translation2d t0 = new Translation2d(.6, .6);
        public Translation2d t1 = new Translation2d(1, 1);
        public Translation2d t2 = new Translation2d(1.1, 1.1);
        public Translation2d t3 = new Translation2d(1, 1.1);

        // Cube
        public ArmAngles midGoalCube = new ArmAngles(0.089803, 1.681915);
        public ArmAngles lowGoalCube = new ArmAngles(-0.049849, 2.271662);
        public ArmAngles subCube = new ArmAngles(-0.341841, 1.361939);
        public ArmAngles subToCube = new ArmAngles(-0.341841, 1.361939);
        public ArmAngles highGoalCube = new ArmAngles(0.316365, 1.147321);

        public ArmAngles safeBack = new ArmAngles(-0.55, 1.97);
        public ArmAngles safeGoalCone = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeGoalCube = new ArmAngles(-0.639248, 1.838205);
        public ArmAngles safeWaypoint = new ArmAngles(-0.394089, 1.226285);
    }

    private final TrajectoryConfig trajecConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        trajecConfig = config;
    }

    public Trajectory makeTrajectory(Translation2d start, Translation2d end) {
        Translation2d e = end.minus(start);
        double angle = degreesFromTranslation2d(e);
        return onePoint(start, end, angle,angle);
    }

    private double degreesFromTranslation2d(Translation2d xy) {
        double x = xy.getX();
        double y = xy.getY();
        double constant = 0;
        if (x <0) {
            constant += 180;
        }
        double atan = Math.atan(y/x);
        double angle = Units.radiansToDegrees(atan)+constant;
        return angle;
    }

    /** from current location to an endpoint */
    public Trajectory onePoint(Translation2d start, Translation2d end, double firstDegree, double secondDegree) {
        return withList(start, List.of(), end, firstDegree, secondDegree);
    }

    // /** from current location, through a waypoint, to an endpoint */
    public Trajectory twoPoint(Translation2d start, Translation2d mid, Translation2d end,
    double firstDegree, double secondDegree) {
    return withList(start, List.of(mid), end,
    firstDegree, secondDegree);
    }

    public  Trajectory  fivePoint(Translation2d start, Translation2d mid1, Translation2d mid2, Translation2d mid3, Translation2d mid4,
    Translation2d end, double firstDegree, double secondDegree) {
        List<Translation2d> list = List.of(mid1,mid2,mid3,mid4);
            return withList(start, list, end, firstDegree, secondDegree);
    }

    private Trajectory withList(Translation2d start, List<Translation2d> list, Translation2d end, double firstDegree, double secondDegree) {
        System.out.println("Start lower theta: " + start.getX() + " Start upper theta: " + start.getY());
        System.out.println("End lower theta: " + end.getX() + " End upper theta: " + end.getY());
        try {
            return TrajectoryGenerator.generateTrajectory(setPose(start, firstDegree), list, setPose(end, secondDegree),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            e.printStackTrace();
            return null;}
    }

    // note proximal is y
    private Pose2d setPose(Translation2d start, double degrees) {
        return new Pose2d(start.getX(), start.getY(), Rotation2d.fromDegrees(degrees));
    }

}
