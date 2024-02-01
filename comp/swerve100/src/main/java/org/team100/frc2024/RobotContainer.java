package org.team100.frc2024;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.motion.IntakeNote;
import org.team100.frc2024.motion.OuttakeNote;
import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.amp.PivotAmp;
import org.team100.frc2024.motion.amp.PivotToAmpPosition;
import org.team100.frc2024.motion.indexer.IndexCommand;
import org.team100.frc2024.motion.indexer.IndexerSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.intake.IntakeFactory;
import org.team100.frc2024.motion.shooter.Shooter;
import org.team100.frc2024.motion.shooter.ShooterFactory;
import org.team100.lib.commands.drivetrain.CommandMaker;
import org.team100.lib.commands.drivetrain.DrawSquare;
import org.team100.lib.commands.drivetrain.DriveInACircle;
import org.team100.lib.commands.drivetrain.DriveInALittleSquare;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.DriveToWaypoint100;
import org.team100.lib.commands.drivetrain.DriveToWaypoint3;
import org.team100.lib.commands.drivetrain.FancyTrajectory;
import org.team100.lib.commands.drivetrain.FullStateTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.ManualMode;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.commands.drivetrain.PermissiveTrajectoryListCommand;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.Rotate;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.Spin;
import org.team100.lib.commands.drivetrain.TrajectoryListCommand;
import org.team100.lib.commands.telemetry.MorseCodeBeep;
import org.team100.lib.config.AllianceSelector;
import org.team100.lib.config.AutonSelector;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.controller.DrivePursuitController;
import org.team100.lib.controller.DriveRamseteController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.Blip24ArrayListener;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.HeadingFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Annunciator;
import org.team100.lib.telemetry.Monitor;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.StraightLineTrajectory;
import org.team100.lib.trajectory.TrajectoryMaker;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.util.Names;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private static final double kDriveCurrentLimit = 60;
    private final Telemetry t = Telemetry.get();

    private final AutonSelector m_autonSelector;
    private final int m_autonRoutine;
    private final AllianceSelector m_allianceSelector;
    private final Alliance m_alliance;

    final HeadingInterface m_heading;
    private final LEDIndicator m_indicator;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    final SwerveDriveSubsystem m_drive;
    private final SwerveModuleCollection m_modules;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final HolonomicDriveController3 m_controller; 
    private final TrajectoryPlanner m_planner;




    private final Command m_auton;

    private final SelfTestRunner m_selfTest;
    final DriveInALittleSquare m_driveInALittleSquare;
    final MorseCodeBeep m_beep;
    final Monitor m_monitor;

    // Identity-specific fields
    final IndexerSubsystem m_indexer;
    final AmpSubsystem m_amp;
    // private final ClimberSubsystem m_climber;
    final Shooter m_shooter;
    final Intake m_intake;

    final DriverControl m_driverControl;
    final OperatorControl m_operatorControl;


    // Commands
    private final PivotAmp m_pivotAmp;

    private final String m_name;

    public RobotContainer(TimedRobot robot) throws IOException {    
        m_name = Names.name(this);

        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_controller = new HolonomicDriveController3();
        m_driverControl = new DriverControlProxy(this);
        m_operatorControl = new OperatorControlProxy(this);

        m_planner = new TrajectoryPlanner(m_swerveKinodynamics);



        // these devices only currently exist on the comp bot
        if (Identity.instance == Identity.COMP_BOT) {
            // digital inputs 0, 1, 2, 3.
            m_autonSelector = new AutonSelector();
            m_autonRoutine = m_autonSelector.routine();
            // digital inputs 4, 5
            m_allianceSelector = new AllianceSelector();
            m_alliance = m_allianceSelector.alliance();
        } else {
            m_autonSelector = null;
            m_autonRoutine = 0;
            m_allianceSelector = null;
            m_alliance = Alliance.Blue;
        }
        if (m_alliance == Alliance.Blue) {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-studies.json");
        } else {
            m_layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-studies.json");
        }
        t.log(Level.INFO, m_name, "Routine", m_autonRoutine);
        t.log(Level.INFO, m_name, "Alliance", m_alliance);

        m_indicator = new LEDIndicator(8);

        // 20 words per minute is 60 ms.
        m_beep = new MorseCodeBeep(0.06);
        // m_beep = new Beep();
        BooleanSupplier test = () -> m_driverControl.annunicatorTest() || m_beep.getOutput();

        m_monitor = new Monitor(new Annunciator(6), test);
        // The monitor runs less frequently than the control loop.
        robot.addPeriodic(m_monitor::periodic, 0.2);

        
        m_modules = SwerveModuleCollection.get(kDriveCurrentLimit, m_swerveKinodynamics);

        m_heading = HeadingFactory.get(m_swerveKinodynamics, m_modules);

        SwerveDrivePoseEstimator poseEstimator = m_swerveKinodynamics.newPoseEstimator(
                m_heading.getHeadingNWU(),
                m_modules.positions(),
                GeometryUtil.kPoseZero,
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.1, 0.1, 0.4));

    
        VisionDataProvider visionDataProvider = new VisionDataProvider(
                m_layout,
                poseEstimator,
                poseEstimator::getEstimatedPosition);
        visionDataProvider.enable();
        
        NotePosition24ArrayListener notePositionDetector = new NotePosition24ArrayListener();
        notePositionDetector.enable();

        Blip24ArrayListener listener = new Blip24ArrayListener();
        listener.enable();

        SwerveLocal swerveLocal = new SwerveLocal(m_swerveKinodynamics, m_modules);

        m_drive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                swerveLocal,
                m_driverControl::speed);

        m_driveInALittleSquare = new DriveInALittleSquare(m_drive);

                
        m_intake = IntakeFactory.get();
        m_shooter = ShooterFactory.get();

        m_indexer = new IndexerSubsystem(30); // NEED CAN FOR AMP MOTOR //5
        m_amp = new AmpSubsystem(28, 39);
        m_pivotAmp = new PivotAmp(m_amp, m_operatorControl::ampPosition);        

       
        bindDriverControls();

        bindOperatorControls();
       


        // TODO: spin up the shooter whenever the robot is in range.
       

        /*
         * 
         * this is another way to do speed control
         * 
         * final double kMaxShooterVelocity = 30.0;
         * m_shooter.setDefaultCommand(
         * m_shooter.run(
         * () -> m_shooter.setVelocity(
         * kMaxShooterVelocity * operatorControl.shooterSpeed())));
         * 
         * the midi control can do it too
         * 
         * m_shooter.setDefaultCommand(
         * m_shooter.run(
         * () -> m_shooter.setVelocity(
         * kMaxShooterVelocity * thirdControl.shooterSpeed())));
         */

        // TODO: intake whenever intake is running.
        // TODO: stop when note is accpeted using optical detector.
        // TODO: shoot only when the shooter is ready.

        


        ///////////////////////////
        //
        // DRIVE
        //

        PIDController thetaController = new PIDController(1.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(0.5, 0, 0);

        m_drive.setDefaultCommand(
                new DriveManually(
                        new ManualMode(),
                        m_driverControl::twist,
                        m_drive,
                        m_heading,
                        m_swerveKinodynamics,
                        m_driverControl::desiredRotation,
                        thetaController,
                        omegaController,
                        m_driverControl::target,
                        m_driverControl::trigger));

        m_intake.setDefaultCommand(m_intake.run(m_intake::stop));
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));
        m_amp.setDefaultCommand(m_pivotAmp);

        m_auton = m_drive.runInit(m_drive::defense);
        // selftest uses fields we just initialized above, so it comes last.
        m_selfTest = new SelfTestRunner(this, m_operatorControl::selfTestEnable);
    }

    public void scheduleSelfTest() {
        m_selfTest.schedule();
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void cancelAuton() {
        if (m_auton == null)
            return;
        m_auton.cancel();
    }

    public double getRoutine() {
        return m_autonSelector.routine();
    }

    public void ledStart() {
        m_indicator.set(State.ORANGE);
    }

    public void ledStop() {
        m_indicator.close();
    }

    public void red() {
        m_indicator.set(State.RED);
    }

    public void green() {
        m_indicator.set(State.GREEN);
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        if (m_autonSelector != null)
            m_autonSelector.close();
        if (m_allianceSelector != null)
            m_allianceSelector.close();
        m_indicator.close();
        m_modules.close();
    }

    public void bindDriverControls(){
        m_driverControl.test().whileTrue(CommandMaker.choreo(Choreo.getTrajectory("fourpiecev3"), m_drive));

        m_driverControl.defense().whileTrue(m_drive.runInit(m_drive::defense));
        m_driverControl.steer0().whileTrue(m_drive.runInit(m_drive::steer0));
        m_driverControl.steer90().whileTrue(m_drive.runInit(m_drive::steer90));

        m_driverControl.resetRotation0().onTrue(new SetRotation(m_drive, GeometryUtil.kRotationZero));
        m_driverControl.resetRotation180().onTrue(new SetRotation(m_drive, Rotation2d.fromDegrees(180)));


        // m_driverControl.resetPose().onTrue(new ResetPose(m_drive, 0.8725617527961731, 3.3582773208618164, 0));
        m_driverControl.resetPose().onTrue(new ResetPose(m_drive, 0.8725617527961731, 3.35, 0));


        m_driverControl.rotate0().whileTrue(new Rotate(m_drive, m_heading, m_swerveKinodynamics, 0));

        m_driverControl.circle().whileTrue(new DrawSquare(m_drive, m_swerveKinodynamics, m_controller));


        m_driverControl.driveWithFancyTrajec().whileTrue(
                new FancyTrajectory(m_drive, m_planner, m_swerveKinodynamics));

        m_driverControl.never().whileTrue(new DriveInACircle(m_drive, m_controller, -1));
        m_driverControl.never().whileTrue(new Spin(m_drive, m_controller));
        m_driverControl.never().whileTrue(new Oscillate(m_drive));

        // make a one-meter line
        m_driverControl.never().whileTrue(
                new TrajectoryListCommand(m_drive, m_controller,
                        x -> List.of(TrajectoryMaker.line(m_swerveKinodynamics, x))));

        // make a one-meter square
        m_driverControl.never().whileTrue(
                new TrajectoryListCommand(m_drive, m_controller,
                        x -> TrajectoryMaker.square(m_swerveKinodynamics, x)));

        // one-meter square with reset at the corners
        m_driverControl.never().whileTrue(
                new PermissiveTrajectoryListCommand(m_drive, m_controller,
                        TrajectoryMaker.permissiveSquare(m_swerveKinodynamics)));

        // one-meter square with position and velocity feedback control
        m_driverControl.never().whileTrue(
                new FullStateTrajectoryListCommand(m_drive,
                        x -> TrajectoryMaker.square(m_swerveKinodynamics, x)));

        // trying the new ChoreoLib
        m_driverControl.never().whileTrue(CommandMaker.choreo(Choreo.getTrajectory("test"), m_drive));

        // playing with trajectory followers
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        StraightLineTrajectory maker = new StraightLineTrajectory(config);

        // field center, roughly, facing to the left.
        Pose2d goal = new Pose2d(8, 4, GeometryUtil.kRotation90);
        Command follower = new DriveToWaypoint3(goal, m_drive, maker, m_controller);
        m_driverControl.never().whileTrue(follower);

        // 254 PID follower
        DriveMotionController drivePID = new DrivePIDFController(false);
        m_driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, m_planner, drivePID, m_swerveKinodynamics));

        // 254 FF follower
        DriveMotionController driveFF = new DrivePIDFController(true);
        m_driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, m_planner, driveFF, m_swerveKinodynamics));

        // 254 Pursuit follower
        DriveMotionController drivePP = new DrivePursuitController(m_swerveKinodynamics);
        m_driverControl.actualCircle().whileTrue(
                new DriveToWaypoint100(goal, m_drive, m_planner, drivePP, m_swerveKinodynamics));

        // 254 Ramsete follower
        // this one seems to have a pretty high tolerance?
        DriveMotionController driveRam = new DriveRamseteController();
        m_driverControl.never().whileTrue(
                new DriveToWaypoint100(goal, m_drive, m_planner, driveRam, m_swerveKinodynamics));

        // little square
        m_driverControl.never().whileTrue(new DriveInALittleSquare(m_drive));

    }

    public void bindOperatorControls(){
        m_operatorControl.intake().whileTrue(new IntakeNote(m_intake, m_indexer));

        m_operatorControl.outtake().whileTrue(new OuttakeNote(m_intake, m_indexer));

        m_operatorControl.pivotToAmpPosition().whileTrue(new PivotToAmpPosition(m_amp));

        m_operatorControl.shooter().whileTrue(m_shooter.run(m_shooter::forward));


    } 

}
