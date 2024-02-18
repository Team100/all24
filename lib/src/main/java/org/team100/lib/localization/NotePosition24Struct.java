package org.team100.lib.localization;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class NotePosition24Struct implements Struct<NotePosition24> {

    @Override
    public Class<NotePosition24> getTypeClass() {
        return NotePosition24.class;
    }

    @Override
    public String getTypeString() {
        return "struct:NotePosition24";
    }

    @Override
    public int getSize() {
        return kSizeFloat + kSizeFloat;
    }

    @Override
    public String getSchema() {
        return "float yaw,float pitch";
    }

    @Override
    public NotePosition24 unpack(ByteBuffer bb) {
        float x = bb.getFloat();
        float y = bb.getFloat();
        return new NotePosition24(x,y);
    }

    @Override
    public void pack(ByteBuffer bb, NotePosition24 value) {
        bb.putFloat(value.getYaw());
        bb.putFloat(value.getPitch());
    }

}
