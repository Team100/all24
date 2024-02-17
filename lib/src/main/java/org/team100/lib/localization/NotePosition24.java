package org.team100.lib.localization;

public class NotePosition24 {
    private final int yaw;
    private final int pitch;

    // Yaw and Pitch angles to object in camera
    public NotePosition24(int yaw, int pitch) {
        this.yaw = yaw;
        this.pitch = pitch;
    }

    public int getYaw() {
        return this.yaw;
    }

    public int getPitch() {
        return this.pitch;
    }

    @Override
    public String toString() {
        return "NotePosition [yaw=" + yaw + ", pitch=" + pitch + "]";
    }

    public static final NotePosition24Struct struct = new NotePosition24Struct();
}
