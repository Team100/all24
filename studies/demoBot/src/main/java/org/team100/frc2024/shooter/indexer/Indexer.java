package org.team100.frc2024.shooter.indexer;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Indexer extends Glassy, Subsystem {

    public void set(double value);

    public void stop();
} 