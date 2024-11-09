package org.team100.lib.util.struct;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.util.OptionalRotation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;

public class OptionalRotation2dStruct implements Struct<OptionalRotation2d> {
    @Override
    public Class<OptionalRotation2d> getTypeClass() {
        return OptionalRotation2d.class;
    }

    @Override
    public String getTypeString() {
        return "struct:OptionalRotation2d";
    }

    @Override
    public int getSize() {
        return kSizeBool + Rotation2d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "boolean present;Rotation2d value";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { Rotation2d.struct };
    }

    @Override
    public OptionalRotation2d unpack(ByteBuffer bb) {
        // bool decoded as byte
        boolean present = bb.get() > 0;
        if (present) {
            Rotation2d value = Rotation2d.struct.unpack(bb);
            return OptionalRotation2d.of(value);
        }
        return OptionalRotation2d.empty();
    }

    @Override
    public void pack(ByteBuffer bb, OptionalRotation2d value) {
        // bool encoded as byte, empty is zero rotation
        Optional<Rotation2d> optional = value.get();
        if (optional.isPresent()) {
            bb.put((byte) 0x01);
            Rotation2d.struct.pack(bb, optional.get());
        } else {
            bb.put((byte) 0x00);
            Rotation2d.struct.pack(bb, GeometryUtil.kRotationZero);

        }
    }
}
