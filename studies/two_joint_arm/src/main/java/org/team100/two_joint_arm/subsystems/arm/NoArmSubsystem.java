package org.team100.two_joint_arm.subsystems.arm;

import org.team100.lib.motion.arm.ArmAngles;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class NoArmSubsystem extends Subsystem implements ArmInterface {

    @Override
    public Subsystem subsystem() {
        return this;
    }

    @Override
    public boolean getCubeMode() {
        return false;
    }

    @Override
    public void setCubeMode(boolean b) {
        //
    }

    @Override
    public void setReference(ArmAngles reference) {
        //
    }

    @Override
    public ArmAngles getMeasurement() {
        return new ArmAngles(0, 0);
    }

    @Override
    public void setControlNormal() {
        //
    }

    @Override
    public void setControlSafe() {
        //
    }

    @Override
    public void close() {
        //
    }
}