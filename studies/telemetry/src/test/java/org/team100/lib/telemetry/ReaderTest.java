package org.team100.lib.telemetry;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

public class ReaderTest {
    @Test
    void testReader() throws IOException {
        System.out.println(Paths.get(".").toAbsolutePath().normalize().toString());
        DataLogReader reader = new DataLogReader("FRC_20231024_161517.wpilog");
        Map<Integer, DataLogRecord.StartRecordData> entries = new HashMap<>();
        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                DataLogRecord.StartRecordData data = record.getStartData();
                entries.put(data.entry, data);
                System.out.println("entry " + data.entry + " " + data.name);
            } else if (record.isFinish()) {
            } else if (record.isSetMetadata()) {
            } else if (record.isControl()) {
            } else {
                // System.out.print("Data(" + record.getEntry() + ", size=" + record.getSize() +
                // ") ");
                DataLogRecord.StartRecordData entry = entries.get(record.getEntry());
                if ("double".equals(entry.type)) {
                    System.out.println("double " + entry.name + " " + record.getDouble());
                }
                if ("int64".equals(entry.type)) {
                    System.out.println("int " + entry.name + " " + record.getInteger());
                }
            }
        }
    }

}
