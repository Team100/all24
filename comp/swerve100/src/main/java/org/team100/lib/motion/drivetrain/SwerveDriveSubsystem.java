package org.team100.lib.motion.drivetrain;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.controller.HolonomicDriveController2;
import org.team100.lib.controller.HolonomicDriveRegulator;
import org.team100.lib.controller.PidGains;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveSubsystem extends Subsystem implements SwerveDriveSubsystemInterface {
    private final Telemetry t = Telemetry.get();

    private final Heading m_heading;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;
    private final FrameTransform m_frameTransform;
    private final SwerveLocal m_swerveLocal;
    private final HolonomicDriveController2 m_controller;
    private final HolonomicDriveRegulator m_regulator = new HolonomicDriveRegulator();
    private ChassisSpeeds m_targetChasis = new ChassisSpeeds();
    // TODO: this looks broken
    public double keyList = -1;

    private SwerveState m_desiredState;

    public SwerveDriveSubsystem(
            Heading heading,
            SwerveDrivePoseEstimator poseEstimator,
            FrameTransform frameTransform,
            SwerveLocal swerveLocal,
            HolonomicDriveController2 controller,
            Field2d field) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_field = field;
        m_frameTransform = frameTransform;
        m_swerveLocal = swerveLocal;
        m_controller = controller;
        truncate();
        t.log("/field/.type", "Field2d");
    }

    /** Set desired states to current states and stop. */
    public void truncate() {
        stop();
        Pose2d currentPose = getPose();
        m_desiredState = new SwerveState(
                new State100(currentPose.getX(), 0, 0),
                new State100(currentPose.getY(), 0, 0),
                new State100(currentPose.getRotation().getRadians(), 0, 0));
    }

    /** Drive to the desired reference. */
    @Override
    public void periodic() {
        updateOdometry();
        driveToReference2();
        m_field.setRobotPose(getPose());
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        return m_swerveLocal.speeds();
    }

    /**
     * Give the controller a new reference state.
     */
    public void 
    
    
    setDesiredState(SwerveState desiredState) {
        m_desiredState = desiredState;
    }

    public void setGains(PidGains cartesian, PidGains rotation) {
        m_controller.setGains(cartesian, rotation);
    }

    public void setIRange(double cartesian) {
        m_controller.setIRange(cartesian);
    }

    public void setTolerance(double cartesian, double rotation) {
        m_controller.setTolerance(cartesian, rotation);
    }

    // this is for testing
    public SwerveDriveSubsystem get() {
        return this;
    }

    private void updateOdometry() {
        m_poseEstimator.update(m_heading.getHeadingNWU(), m_swerveLocal.positions());
        // {
        // if (m_pose.aprilPresent()) {
        // m_poseEstimator.addVisionMeasurement(
        // m_pose.getRobotPose(0),
        // Timer.getFPGATimestamp() - 0.3);
        // }

        // Update the Field2d widget
        Pose2d newEstimate = getPose();
        t.log("/field/robotPose", new double[] {
                newEstimate.getX(),
                newEstimate.getY(),
                newEstimate.getRotation().getDegrees()
        });
        t.log("/current pose/x m", newEstimate.getX());
        t.log("/current pose/y m", newEstimate.getY());
        t.log("/current pose/theta rad", newEstimate.getRotation().getRadians());
        t.log("/current pose/Heading NWU rad_s", m_heading.getHeadingRateNWU());
    }

    private void driveToReference() {
        // TODO: pose should be a full state, with velocity and acceleration.
        Pose2d currentPose = getPose();

        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, m_desiredState);
        driveInFieldCoords(fieldRelativeTarget);
    }

    private void driveToReference2() {
        // TODO: pose should be a full state, with velocity and acceleration.
        Pose2d currentPose = getPose();

        Twist2d fieldRelativeTarget = m_regulator.calculate(currentPose, m_desiredState);
        driveInFieldCoords(fieldRelativeTarget);
    }

    ////////////////////////////
    // TODO: push the stuff below down

    /**
     * @param twist Field coordinate velocities in meters and radians per second.
     */
    private void driveInFieldCoords(Twist2d twist) {
        ChassisSpeeds targetChassisSpeeds = m_frameTransform.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, getPose().getRotation());

        // System.out.println(targetChassisSpeeds);

        m_targetChasis = targetChassisSpeeds;

        t.log("/chassis/x m", twist.dx);
        t.log("/chassis/y m", twist.dy);
        t.log("/chassis/theta rad", twist.dtheta);

        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        m_swerveLocal.setChassisSpeeds(speeds);
    }

    /**
     * Helper for incremental driving.
     * 
     * Note the returned state has zero acceleration, which is wrong.
     * 
     * TODO: correct acceleration.
     * 
     * @param twistM_S incremental input in m/s and rad/s
     * @return SwerveState representing 0.02 sec of twist applied to the current
     *         pose.
     */
    public static SwerveState incremental(Pose2d currentPose, Twist2d twistM_S) {
        Twist2d twistM = DriveUtil.scale(twistM_S, 0.02, 0.02);
        Pose2d ref = currentPose.exp(twistM);
        return new SwerveState(
                new State100(ref.getX(), twistM_S.dx, 0),
                new State100(ref.getY(), twistM_S.dy, 0),
                new State100(ref.getRotation().getRadians(), twistM_S.dtheta, 0));

    }

    public void defense() {
        m_swerveLocal.defense();
    }

    public void test(double[][] desiredOutputs) {
        m_swerveLocal.test(desiredOutputs);
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    ////////////////////////////////////////
    // pose estimator stuff

    /**
     * Note this doesn't include the gyro reading directly, the estimate is
     * considerably massaged by the odometry logic.
     */
    @Override
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(m_heading.getHeadingNWU(), m_swerveLocal.positions(), robotPose);
    }

    ///////////////////////////////
    // TODO: move this somewhere else

    // TODO: this looks broken
    public void setKeyList() {
        keyList = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
    }

    // TODO: this looks broken
    public double getVisionSize() {
        return keyList;
    }
}
