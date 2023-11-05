package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An attempt to navigate to a waypoint using X, Y, and theta controllers.
 * 
 * This isn't done and doesn't work well, you shouldn't use it.
 * 
 * The optimal 3d path is not decomposable this way.
 * 
 * This produces garbage paths.
 */
public class DriveToWaypoint extends Command {
    private static final double kPeriodS = 0.02;

    private static final double kMaxSpeedM_s = 3.0;
    private static final double kMaxAccelM_s_s = 6.0;

    private static final double kMaxAngularSpeedRad_s = 3.0 * Math.PI;
    private static final double kMaxAngularAccelRad_s_s = 6.0 * Math.PI;

    private static final double xyP = 0;
    private static final double xyD = 0;
    private static final double thetaP = 0;
    private static final double thetaD = 0;

    // private static final double xySP = 0;
    // private static final double xySD = 0;
    // private static final double thetaSP = 0;
    // private static final double thetaSD = 0;

    private static final double goalSpeed = 0.0; // for now goals are always stationary.

    // 1.0 => try to drive at the setpoint velocity
    private static final double xKV = 1.0;
    // turn off y for now
    private static final double yKV = 0.9;
    // turn off theta for now
    private static final double thetaKV = 0.9;

    private final Supplier<Pose2d> waypointSupplier;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<SwerveModuleState[]> outputModuleStateConsumer;

    // these try to reach the profile
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    // private final PIDController xSpeedController;
    // private final PIDController ySpeedController;
    // private final PIDController thetaSpeedController;

    private Pose2d previousPose = null;
    private int t = 0;

    /**
     * @param waypointSupplier          the destination
     * @param poseSupplier              current robot pose from estimator
     * @param outputModuleStateConsumer control output
     * @param requirement               for the scheduler
     */
    public DriveToWaypoint(
            Supplier<Pose2d> waypointSupplier,
            Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics,
            Consumer<SwerveModuleState[]> outputModuleStateConsumer,
            Drivetrain requirement) {
        this.waypointSupplier = waypointSupplier;
        this.poseSupplier = poseSupplier;
        this.kinematics = kinematics;
        this.outputModuleStateConsumer = outputModuleStateConsumer;
        addRequirements(requirement);

        xController = new PIDController(xyP, 0, xyD);
        yController = new PIDController(xyP, 0, xyD);
        thetaController = new PIDController(thetaP, 0, thetaD);

        // xSpeedController = new PIDController(xySP, 0, xySD);
        // ySpeedController = new PIDController(xySP, 0, xySD);
        // thetaSpeedController = new PIDController(thetaSP, 0, thetaSD);
    }

    @Override
    public void execute() {
        Pose2d waypoint = waypointSupplier.get();
        Pose2d pose = poseSupplier.get();

        double xMaxSpeedM_s = kMaxSpeedM_s;
        double yMaxSpeedM_s = kMaxSpeedM_s;
        double xMaxAccelM_s_s = kMaxAccelM_s_s;
        double yMaxAccelM_s_s = kMaxAccelM_s_s;

        TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(xMaxSpeedM_s, xMaxAccelM_s_s);
        TrapezoidProfile xProfile = new TrapezoidProfile(xConstraints);

        TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(yMaxSpeedM_s, yMaxAccelM_s_s);
        TrapezoidProfile yProfile = new TrapezoidProfile(yConstraints);

        TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRad_s,
                kMaxAngularAccelRad_s_s);
        TrapezoidProfile thetaProfile = new TrapezoidProfile(thetaConstraints);

        if (previousPose == null) {
            // first time, velocity is zero
            previousPose = pose;
        }

        // for now use "city block metric". TODO: correct for diagonals

        // X
        TrapezoidProfile.State xGoal = new TrapezoidProfile.State(waypoint.getX(), goalSpeed);
        double xMeasurementM = pose.getX();
        double xSpeedM_s = (xMeasurementM - previousPose.getX()) / kPeriodS;
        TrapezoidProfile.State xInitial = new TrapezoidProfile.State(xMeasurementM, xSpeedM_s);
        TrapezoidProfile.State xSetpoint = xProfile.calculate(kPeriodS, xGoal, xInitial);
        double xFFM_s = xSetpoint.velocity * xKV;
        if (xProfile.isFinished(kPeriodS)) {
            xFFM_s = 0;
        }

        // double xFFM_s = 6.0;
        double xOutputM_s = xController.calculate(xMeasurementM, xSetpoint.position);
        // double xFFM_s = xSpeedController.calculate(xSpeedM_s, xSetpoint.velocity);
        double xErrM = xController.getPositionError();
        double xVErrM_s = xController.getVelocityError();

        // Y
        TrapezoidProfile.State yGoal = new TrapezoidProfile.State(waypoint.getY(), goalSpeed);
        double yMeasurementM = pose.getY();
        double ySpeedM_s = (yMeasurementM - previousPose.getY()) / kPeriodS;
        TrapezoidProfile.State yInitial = new TrapezoidProfile.State(yMeasurementM, ySpeedM_s);
        TrapezoidProfile.State ySetpoint = yProfile.calculate(kPeriodS, yGoal, yInitial);
        double yFFM_s = ySetpoint.velocity * yKV;
        if (yProfile.isFinished(kPeriodS)) {
            yFFM_s = 0;
        }
        double yOutputM_s = yController.calculate(yMeasurementM, ySetpoint.position);
        // double yFFM_s = ySpeedController.calculate(ySpeedM_s, ySetpoint.velocity);
        double yErrM = yController.getPositionError();
        double yVErrM_s = yController.getVelocityError();

        // THETA
        TrapezoidProfile.State thetaGoal = new TrapezoidProfile.State(waypoint.getRotation().getRadians(), goalSpeed);
        double thetaMeasurementRad = pose.getRotation().getRadians();
        double thetaSpeedRad_s = (thetaMeasurementRad - previousPose.getRotation().getRadians()) / kPeriodS;
        TrapezoidProfile.State thetaInitial = new TrapezoidProfile.State(thetaMeasurementRad, thetaSpeedRad_s);
        TrapezoidProfile.State thetaSetpoint = thetaProfile.calculate(kPeriodS, thetaGoal, thetaInitial);
        double thetaFFRad_s = thetaSetpoint.velocity * thetaKV;
        if (thetaProfile.isFinished(kPeriodS)) {
            thetaFFRad_s = 0;
        }
        double thetaOutputRad_s = thetaController.calculate(thetaMeasurementRad, thetaSetpoint.position);
        // double thetaFFRad_s = thetaSpeedController.calculate(thetaSpeedRad_s,
        // thetaSetpoint.velocity);
        double thetaErrRad = thetaController.getPositionError();
        double thetaVErrRad_s = thetaController.getVelocityError();

        previousPose = pose;

        System.out.printf("t %4.2f GOALS[x: %5.2f y %5.2f theta %5.2f] ",
                t * kPeriodS, xGoal.position, yGoal.position, thetaGoal.position);
        System.out.printf("INITIAL[POSITION[x %5.2f y %5.2f theta %5.2f]",
                xInitial.position, yInitial.position, thetaInitial.position);
        System.out.printf("VELOCITY[x %5.2f y %5.2f theta %5.2f]]",
                xInitial.velocity, yInitial.velocity, thetaInitial.velocity);
        System.out.printf("SETPOINT[POSITION[x %5.2f y %5.2f theta %5.2f] ",
                xSetpoint.position, ySetpoint.position, thetaSetpoint.position);
        System.out.printf("VELOCITY[x %5.2f y %5.2f theta %5.2f]]",
                xSetpoint.velocity, ySetpoint.velocity, thetaSetpoint.velocity);
        System.out.printf("ERROR[POSITION[x %5.2f y %5.2f theta %5.2f] ",
                xErrM, yErrM, thetaErrRad);
        System.out.printf("VELOCITY[x %5.2f y %5.2f theta %6.2f]]",
                xVErrM_s, yVErrM_s, thetaVErrRad_s);
        System.out.printf("OUTPUT[FF[x %5.2f y %5.2f theta %5.2f]",
                xFFM_s, yFFM_s, thetaFFRad_s);
        System.out.printf("CONTROL[x %5.2f y %5.2f theta %5.2f]",
                xOutputM_s, yOutputM_s, thetaOutputRad_s);
        System.out.println();

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFFM_s + xOutputM_s,
                yFFM_s + yOutputM_s,
                thetaFFRad_s + thetaOutputRad_s,
                pose.getRotation());
        SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
        outputModuleStateConsumer.accept(targetModuleStates);
        t += 1;
    }

    @Override
    public boolean isFinished() {
        return false; // controllers at goals, measurements at setpoints
    }
}
