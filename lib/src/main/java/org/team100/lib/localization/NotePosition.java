package org.team100.lib.localization;

import edu.wpi.first.math.geometry.Translation2d;

public class NotePosition {
    Translation2d pose;
    public NotePosition(Translation2d pose) {
        this.pose = pose;
    }
    public Translation2d getPose() {
        return pose;
    }
    public static final NotePositionStruct struct = new NotePositionStruct();
}

