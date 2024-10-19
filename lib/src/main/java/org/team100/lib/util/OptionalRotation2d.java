package org.team100.lib.util;

import java.util.Optional;

import org.team100.lib.util.struct.OptionalRotation2dStruct;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.StructSerializable;

/** This is to work around the lack of "missing value" in Structs. */
public class OptionalRotation2d implements StructSerializable {

    private final boolean present;
    private final Rotation2d value;

    public OptionalRotation2d(boolean present, Rotation2d value) {
        this.present = present;
        this.value = value;
    }

    public OptionalRotation2d(Optional<Rotation2d> value) {
        this.present = value.isPresent();
        this.value = value.orElse(null);
    }

    public static OptionalRotation2d of(Rotation2d value) {
        return new OptionalRotation2d(true, value);
    }

    public static OptionalRotation2d empty() {
        return new OptionalRotation2d(false, null);
    }

    public Optional<Rotation2d> get() {
        if (present)
            return Optional.of(value);
        return Optional.empty();
    }

    public static final OptionalRotation2dStruct struct = new OptionalRotation2dStruct();

}
