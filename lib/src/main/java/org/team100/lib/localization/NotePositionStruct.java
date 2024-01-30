package org.team100.lib.localization;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class NotePositionStruct implements Struct<NotePosition> {

    @Override
    public Class<NotePosition> getTypeClass() {
        return NotePosition.class;
    }

    @Override
    public String getTypeString() {
        return "struct:NotePosition";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + kSizeInt32;
    }

    @Override
    public String getSchema() {
        return "int x,int y";
    }

    @Override
    public NotePosition unpack(ByteBuffer bb) {
        int x = bb.getInt(0);
        int y = bb.getInt(1);
        return new NotePosition(x,y);
    }

    @Override
    public void pack(ByteBuffer bb, NotePosition value) {
        bb.putInt(value.getX());
        bb.putInt(value.getY());
    }

}
