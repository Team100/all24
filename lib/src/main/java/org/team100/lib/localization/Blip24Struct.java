package org.team100.lib.localization;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;

public class Blip24Struct implements Struct<Blip24> {

    @Override
    public Class<Blip24> getTypeClass() {
        return Blip24.class;
    }

    @Override
    public String getTypeString() {
        return "struct:Blip24";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + Transform3d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "int id;Transform3d pose";
    }

    @Override
    public Blip24 unpack(ByteBuffer bb) {
        int id = bb.getInt();
        Transform3d pose = Transform3d.struct.unpack(bb);
        return new Blip24(id, pose);
    }

    @Override
    public void pack(ByteBuffer bb, Blip24 value) {
        bb.putInt(value.getId());
        Transform3d.struct.pack(bb, value.getPose());
    }

}
