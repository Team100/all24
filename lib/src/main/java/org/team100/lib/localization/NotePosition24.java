package org.team100.lib.localization;

public class NotePosition24 {
    private final int x;
    private final int y;
    public NotePosition24(int x, int y) {
        this.x = x;
        this.y = y;
    }
    public int getX() {
        return this.x;
    }
    public int getY() {
        return this.y;
    }
    @Override
    public String toString() {
        return "NotePosition [x=" + x + ", y=" + y + "]";
    }

    public static final NotePosition24Struct struct = new NotePosition24Struct();
}

