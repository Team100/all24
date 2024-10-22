package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.struct.SwerveModulePosition100Struct;

class SwerveModulePosition100Test {
    @Test
    void testStruct() {
        SwerveModulePosition100Struct s = SwerveModulePosition100.struct;
        ByteBuffer bb = ByteBuffer.allocate(s.getSize());
        SwerveModulePosition100 p = new SwerveModulePosition100(1, Optional.of(GeometryUtil.kRotationZero));
        s.pack(bb, p);
        assertEquals(0, bb.remaining());
        // distance = 8
        // boolean = 1
        // angle = 8
        assertEquals(17, bb.position());
        bb.rewind();
        SwerveModulePosition100 p2 = s.unpack(bb);
        assertEquals(1.0, p2.distanceMeters, 0.001);
        assertEquals(0.0, p2.angle.get().getRadians(), 0.001);
    }

    @Test
    void testEmpty() {
        SwerveModulePosition100Struct s = SwerveModulePosition100.struct;
        ByteBuffer bb = ByteBuffer.allocate(s.getSize());
        SwerveModulePosition100 p = new SwerveModulePosition100(1, Optional.empty());
        s.pack(bb, p);
        assertEquals(0, bb.remaining());
        assertEquals(17, bb.position());
        bb.rewind();
        SwerveModulePosition100 p2 = s.unpack(bb);
        assertEquals(1.0, p2.distanceMeters, 0.001);
        assertTrue(p2.angle.isEmpty());
    }
}
