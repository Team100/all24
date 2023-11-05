package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.HexFormat;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;

class BlipSerializerTest {
    @Test
    void testBlip() throws StreamReadException, DatabindException, IOException {
        // from the python test
        String hexString = "81a4746167739183a2696401a6706f73" +
                "655f749391cb3ff000000000000091cb" +
                "400000000000000091cb400800000000" +
                "0000a6706f73655f529393cb3ff00000" +
                "00000000cb4000000000000000cb4008" +
                "00000000000093cb4010000000000000" +
                "cb4014000000000000cb401800000000" +
                "000093cb401c000000000000cb402000" +
                "0000000000cb4022000000000000";
        HexFormat hexFormat = HexFormat.of();
        byte[] payload = hexFormat.parseHex(hexString);
        Blips blips = BlipSerializer.deserialize(payload);
        assertEquals(
                "Blips [et=0.0, tags=[Blip [id=1, "
                        + "pose_R=[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]], "
                        + "pose_t=[[1.0], [2.0], [3.0]]]]]",
                blips.toString());
    }

}
