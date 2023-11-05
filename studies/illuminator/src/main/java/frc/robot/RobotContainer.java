package frc.robot;

import org.team100.frc2023.commands.retro.DriveToRetroReflectiveTape;
import org.team100.frc2023.control.Control;
import org.team100.lib.commands.retro.LedOn;
import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.retro.Illuminator;
import org.team100.lib.retro.IlluminatorInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotContainer {
    private final Control control;
    private final IlluminatorInterface illuminator;
    private final SwerveDriveSubsystemInterface m_robotDrive;

    public RobotContainer() {
        Identity identity = Identity.get();
        illuminator = new Illuminator.Factory(identity).get(25);
        control = new Control() {
        };
        control.ledOn(new LedOn(illuminator));
        m_robotDrive = new SwerveDriveSubsystemInterface() {

            @Override
            public Pose2d getPose() {
                return null;
            }

            @Override
            public void stop() {
                //
            }

            @Override
            public void setDesiredState(SwerveState desiredState) {
                //
            }

            @Override
            public ChassisSpeeds speeds() {
                return null;
            }

            @Override
            public void truncate() {
                //
            }

            @Override
            public SwerveDriveSubsystemInterface get() {
                return null;
            }
        };
        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, false);
        control.tapeDetect(new DriveToRetroReflectiveTape(m_robotDrive, speedLimits));
    }

    public void close() {
        illuminator.close();
    }
}
