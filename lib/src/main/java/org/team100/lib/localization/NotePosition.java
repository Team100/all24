package org.team100.lib.localization;

public class NotePosition {
    private final int x;
    private final int y;
    public NotePosition(int x, int y) {
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

    public static final NotePositionStruct struct = new NotePositionStruct();
}

