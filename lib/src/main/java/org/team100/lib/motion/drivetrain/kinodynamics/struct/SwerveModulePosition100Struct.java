package org.team100.lib.motion.drivetrain.kinodynamics.struct;

import java.nio.ByteBuffer;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.util.OptionalRotation2d;

import edu.wpi.first.util.struct.Struct;

/**
 * This is a copy of
 * {@link edu.wpi.first.math.kinematics.struct.SwerveModulePositionStruct} but
 * with optional rotation, working around the incorrect behavior of
 * Rotation2d(0, 0). Because structs don't support optional values, it's a bit
 * more pain than you might expect.
 */
public class SwerveModulePosition100Struct implements Struct<SwerveModulePosition100> {

    @Override
    public Class<SwerveModulePosition100> getTypeClass() {
        return SwerveModulePosition100.class;
    }

    @Override
    public String getTypeString() {
        return "struct:SwerveModulePosition100";
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
    public SwerveModulePosition100 unpack(ByteBuffer bb) {
        double distance = bb.getDouble();
        OptionalRotation2d angle = OptionalRotation2d.struct.unpack(bb);
        return new SwerveModulePosition100(distance, angle.get());
    }

    @Override
    public void pack(ByteBuffer bb, SwerveModulePosition100 value) {
        bb.putDouble(value.distanceMeters);
        OptionalRotation2d.struct.pack(bb, new OptionalRotation2d(value.angle));
    }

}
