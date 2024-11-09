package org.team100.lib.motion.drivetrain.kinodynamics.struct;

import java.nio.ByteBuffer;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleDelta;
import org.team100.lib.util.OptionalRotation2d;

import edu.wpi.first.util.struct.Struct;

public class SwerveModuleDeltaStruct implements Struct<SwerveModuleDelta> {
    @Override
    public Class<SwerveModuleDelta> getTypeClass() {
        return SwerveModuleDelta.class;
    }

    @Override
    public String getTypeString() {
        return "struct:SwerveModuleDelta";
    }

    @Override
    public int getSize() {
        return kSizeDouble + OptionalRotation2d.struct.getSize();
    }

    @Override
    public String getSchema() {
        return "double distance;OptionalRotation2d angle";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] { OptionalRotation2d.struct };
    }

    @Override
    public SwerveModuleDelta unpack(ByteBuffer bb) {
        double distance = bb.getDouble();
        OptionalRotation2d angle = OptionalRotation2d.struct.unpack(bb);
        return new SwerveModuleDelta(distance, angle.get());
    }

    @Override
    public void pack(ByteBuffer bb, SwerveModuleDelta value) {
        bb.putDouble(value.distanceMeters);
        OptionalRotation2d.struct.pack(bb, new OptionalRotation2d(value.angle));
    }
}
