package org.team100.lib.localization;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Translation2d;
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
        return kSizeInt32 + Translation2d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "int id;Translation2d pose";
    }

    @Override
    public NotePosition unpack(ByteBuffer bb) {
        Translation2d pose = Translation2d.struct.unpack(bb);
        return new NotePosition(pose);
    }

    @Override
    public void pack(ByteBuffer bb, NotePosition value) {
        Translation2d.struct.pack(bb, value.getPose());
    }

}
