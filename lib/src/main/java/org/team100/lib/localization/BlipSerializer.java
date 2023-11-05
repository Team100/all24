package org.team100.lib.localization;

import java.io.IOException;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class BlipSerializer {
    public static Blips deserialize(byte[] payload) throws StreamReadException, DatabindException, IOException {
        ObjectMapper object_mapper = new ObjectMapper(new MessagePackFactory());
        return object_mapper.readValue(payload, Blips.class);
    }

}
