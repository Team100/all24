package org.team100.lib.commands.drivetrain.manual;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual drivetrain control.
 * 
 * Provides four manual control modes:
 * 
 * -- raw module state
 * -- robot-relative
 * -- field-relative
 * -- field-relative with rotation control
 * 
 * Use the mode supplier to choose which mode to use, e.g. using a Sendable
 * Chooser.
 */
public class DriveManually extends Command implements Glassy  {

    private static final SendableChooser<String> m_manualModeChooser = new NamedChooser<>("Manual Drive Mode") {
    };

    private Supplier<String> m_mode;
    /**
     * Velocity control in control units, [-1,1] on all axes. This needs to be
     * mapped to a feasible velocity control as early as possible.
     */
    private final Supplier<DriverControl.Velocity> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final Map<String, Driver> m_drivers;
    private final Driver m_defaultDriver;
    String currentManualMode = null;

    public DriveManually(
            Supplier<DriverControl.Velocity> twistSupplier,
            SwerveDriveSubsystem robotDrive) {
        m_mode = m_manualModeChooser::getSelected;
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_defaultDriver = stop();
        m_drivers = new ConcurrentHashMap<>();
        SmartDashboard.putData(m_manualModeChooser);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // the setpoint generator remembers what it was doing before, but it might be
        // interrupted by some other command, so when we start, we have to tell it what
        // the real previous setpoint is.
        // Note this is not necessarily "at rest," because we might start driving
        // manually while the robot is moving.
        ChassisSpeeds currentSpeeds = m_drive.getChassisSpeeds();
        SwerveModuleStates currentStates = m_drive.getSwerveLocal().states();
        SwerveSetpoint setpoint = new SwerveSetpoint(currentSpeeds, currentStates);
        m_drive.resetSetpoint(setpoint);
        Pose2d p = m_drive.getPose();
        for (Driver d : m_drivers.values()) {
            d.reset(p);
        }
    }

    @Override
    public void execute() {
        String manualMode = m_mode.get();
        // System.out.println("manual mode " + manualMode);
        if (manualMode == null) {
            return;
        }

        if (!(manualMode.equals(currentManualMode))) {
            // System.out.println("reset mode");
            currentManualMode = manualMode;
            // there's state in there we'd like to forget
            Pose2d p = m_drive.getPose();
            for (Driver d : m_drivers.values()) {
                d.reset(p);
            }
        }

        // input in [-1,1] control units
        DriverControl.Velocity input = m_twistSupplier.get();
        // System.out.println("input" + input);
        SwerveState state = m_drive.getState();
        Driver d = m_drivers.getOrDefault(manualMode, m_defaultDriver);
        d.apply(state, input);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    /**
     * Override the driver mode.
     * 
     * For testing only.
     */
    void overrideMode(Supplier<String> mode) {
        m_mode = mode;
    }

    /** Register a driver for module state mode */
    public void register(String name, boolean isDefault, ModuleStateDriver d) {
        // System.out.println("register " + name);
        addName(name, isDefault);
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveState s, DriverControl.Velocity t) {
                        // System.out.println("apply t " + t);
                        m_drive.setRawModuleStates(d.apply(t));
                    }

                    public void reset(Pose2d p) {
                        //
                    }
                });
    }

    /** Register a driver for robot-relative speed mode */
    public void register(String name, boolean isDefault, ChassisSpeedDriver d) {
        addName(name, isDefault);
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveState s, DriverControl.Velocity t) {
                        m_drive.setChassisSpeeds(d.apply(s, t));
                    }

                    public void reset(Pose2d p) {
                        d.reset(p);
                    }
                });
    }

    /** Register a driver for field-relative speed mode */
    public void register(String name, boolean isDefault, FieldRelativeDriver d) {
        addName(name, isDefault);
        m_drivers.put(
                name,
                new Driver() {
                    public void apply(SwerveState s, DriverControl.Velocity t) {
                        m_drive.driveInFieldCoords(d.apply(s, t));
                    }

                    public void reset(Pose2d p) {
                        d.reset(p);
                    }
                });
    }

    //////////////

    private Driver stop() {
        return new Driver() {
            public void apply(SwerveState s, DriverControl.Velocity t) {
                m_drive.stop();
            }

            public void reset(Pose2d p) {
                //
            }
        };
    }

    private void addName(String name, boolean isDefault) {
        m_manualModeChooser.addOption(name, name);
        if (isDefault)
            m_manualModeChooser.setDefaultOption(name, name);
    }
}
