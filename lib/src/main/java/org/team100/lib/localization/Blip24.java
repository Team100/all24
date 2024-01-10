package org.team100.lib.localization;

import edu.wpi.first.math.geometry.Transform3d;

/** Mirrors tag_finder24.py Blip24. */
public class Blip24 {
    private final int id;

    private final Transform3d pose;

    public Blip24(int id, Transform3d pose) {
        this.id = id;
        this.pose = pose;
    }

    public int getId() {
        return id;
    }

    public Transform3d getPose() {
        return pose;
    }

    @Override
    public String toString() {
        return "Blip24 [id=" + id + ", pose=" + pose + "]";
    }

    public static final Blip24Struct struct = new Blip24Struct();
}
