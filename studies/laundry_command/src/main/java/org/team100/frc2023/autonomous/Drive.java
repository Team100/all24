package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.LaundryDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Drives straight for the specified speed and duration. */
public class Drive extends Command {
    private final LaundryDrive m_drive;
    private final Timer m_timer;
    private final double m_speed;
    private final double m_duration;

    public Drive(LaundryDrive drive, double speed, double duration) {
        m_drive = drive;
        m_timer = new Timer();
        m_speed = speed;
        m_duration = duration;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.enable();
        m_drive.setSpeeds(m_speed, 0);
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.disable();
    }
}
