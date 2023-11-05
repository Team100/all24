package org.team100.frc2023.commands;

import org.team100.frc2023.control.LaundryStick;
import org.team100.frc2023.subsystems.LaundryArm;
import org.team100.frc2023.subsystems.LaundryDrive;

import edu.wpi.first.wpilibj2.command.Command;

public class Manual extends Command {
    private final LaundryStick m_stick;
    private final LaundryArm m_arm;
    private final LaundryDrive m_drive;

    /** Manual control of drive speeds and arm dumping. */
    public Manual(LaundryStick stick, LaundryArm arm, LaundryDrive drive) {
        m_stick = stick;
        m_arm = arm;
        m_drive = drive;
        addRequirements(arm, drive);
    }

    @Override
    public void initialize() {
        m_arm.enable();
        m_drive.enable();
    }

    @Override
    public void execute() {
        if (m_stick.dump()) {
            m_arm.dump();
        } else {
            m_arm.level();
        }
        m_drive.setSpeeds(m_stick.xSpeed1_1(), m_stick.zSpeed1_1());
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.disable();
        m_drive.disable();
    }
}
