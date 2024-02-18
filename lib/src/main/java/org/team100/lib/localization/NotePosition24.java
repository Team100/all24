package org.team100.lib.localization;

public class NotePosition24 {
    private final float yaw;
    private final float pitch;

    // Yaw and Pitch angles to object in camera
    public NotePosition24(float yaw, float pitch) {
        this.yaw = yaw;
        this.pitch = pitch;
    }

    public float getYaw() {
        return this.yaw;
    }

    public float getPitch() {
        return this.pitch;
    }

    @Override
    public String toString() {
        return "NotePosition [yaw=" + yaw + ", pitch=" + pitch + "]";
    }

    public static final NotePosition24Struct struct = new NotePosition24Struct();
}
