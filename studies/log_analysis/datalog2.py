#! /usr/bin/env python3
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

#
# optimized version of datalog thing
#

import array
import struct
from typing import Any, List, SupportsBytes, Tuple
import msgpack

__all__ = ["StartRecordData", "MetadataRecordData", "DataLogRecord", "DataLogReader"]

floatStruct = struct.Struct("<f")
doubleStruct = struct.Struct("<d")

kControlStart = 0
kControlFinish = 1
kControlSetMetadata = 2


class StartRecordData:
    """Data contained in a start control record as created by DataLog.start() when
    writing the log. This can be read by calling DataLogRecord.getStartData().

    entry: Entry ID; this will be used for this entry in future records.
    name: Entry name.
    type: Type of the stored data for this entry, as a string, e.g. "double".
    metadata: Initial metadata.
    """

    def __init__(self, entry: int, name: str, type: str, metadata: str):
        self.entry = entry
        self.name = name
        self.type = type
        self.metadata = metadata


class MetadataRecordData:
    """Data contained in a set metadata control record as created by
    DataLog.setMetadata(). This can be read by calling
    DataLogRecord.getSetMetadataData().

    entry: Entry ID.
    metadata: New metadata for the entry.
    """

    def __init__(self, entry: int, metadata: str):
        self.entry = entry
        self.metadata = metadata


# now this uses tuples and static methods
class DataLogRecord:
    """A record in the data log. May represent either a control record
    (entry == 0) or a data record."""

    # 0: entry, 1: timestamp, 2: data

    # def __init__(self, entry: int, timestamp: int, data: SupportsBytes):
    #     self.entry = entry
    #     self.timestamp = timestamp
    #     self.data = data

    @staticmethod
    def isControl(self) -> bool:
#        return self.entry == 0
        return self[0] == 0

    @staticmethod
    def _getControlType(self) -> int:
#        return self.data[0]
        return self[2][0]

    @staticmethod
    def isStart(self) -> bool:
        return (
#            self.entry == 0
            self[0] == 0
#            and len(self.data) >= 17
            and len(self[2]) >= 17
#            and self._getControlType() == kControlStart
            and DataLogRecord._getControlType(self) == kControlStart
        )

    @staticmethod
    def isFinish(self) -> bool:
        return (
#            self.entry == 0
            self[0] == 0
#            and len(self.data) == 5
            and len(self[2]) == 5
#            and self._getControlType() == kControlFinish
            and DataLogRecord._getControlType(self) == kControlFinish
        )

    @staticmethod
    def isSetMetadata(self) -> bool:
        return (
#            self.entry == 0
            self[0] == 0
 #           and len(self.data) >= 9
            and len(self[2]) >= 9
  #          and self._getControlType() == kControlSetMetadata
            and DataLogRecord._getControlType(self) == kControlSetMetadata
        )

    @staticmethod
    def getStartData(self) -> StartRecordData:
#        if not self.isStart():
        if not DataLogRecord.isStart(self):
            raise TypeError("not a start record")
#        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        entry = int.from_bytes(self[2][1:5], byteorder="little", signed=False)
#        name, pos = self._readInnerString(5)
        name, pos = DataLogRecord._readInnerString(self, 5)
#        type, pos = self._readInnerString(pos)
        type, pos = DataLogRecord._readInnerString(self,pos)
#        metadata = self._readInnerString(pos)[0]
        metadata = DataLogRecord._readInnerString(self, pos)[0]
        return StartRecordData(entry, name, type, metadata)

    @staticmethod
    def getFinishEntry(self) -> int:
#        if not self.isFinish():
        if not DataLogRecord.isFinish(self):
            raise TypeError("not a finish record")
#        return int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        return int.from_bytes(self[2][1:5], byteorder="little", signed=False)

    @staticmethod
    def getSetMetadataData(self) -> MetadataRecordData:
#        if not self.isSetMetadata():
        if not DataLogRecord.isSetMetadata(self):
            raise TypeError("not a finish record")
#        entry = int.from_bytes(self.data[1:5], byteorder="little", signed=False)
        entry = int.from_bytes(self[2][1:5], byteorder="little", signed=False)
#        metadata = self._readInnerString(5)[0]
        metadata = DataLogRecord._readInnerString(self, 5)[0]
        return MetadataRecordData(entry, metadata)

    @staticmethod
    def getBoolean(self) -> bool:
#        if len(self.data) != 1:
        if len(self[2]) != 1:
            raise TypeError("not a boolean")
#        return self.data[0] != 0
        return self[2][0] != 0

    @staticmethod
    def getInteger(self) -> int:
#        if len(self.data) != 8:
        if len(self[2]) != 8:
            raise TypeError("not an integer")
#        return int.from_bytes(self.data, byteorder="little", signed=True)
        return int.from_bytes(self[2], byteorder="little", signed=True)

    @staticmethod
    def getFloat(self) -> float:
#        if len(self.data) != 4:
        if len(self[2]) != 4:
            raise TypeError("not a float")
#        return floatStruct.unpack(self.data)[0]
        return floatStruct.unpack(self[2])[0]

    @staticmethod
    def getDouble(self) -> float:
#        if len(self.data) != 8:
        if len(self[2]) != 8:
            raise TypeError("not a double")
#        return doubleStruct.unpack(self.data)[0]
        return doubleStruct.unpack(self[2])[0]

    @staticmethod
    def getString(self) -> str:
#        return str(self.data, encoding="utf-8")
        return str(self[2], encoding="utf-8")

    @staticmethod
    def getBooleanArray(self) -> List[bool]:
#        return [x != 0 for x in self.data]
        return [x != 0 for x in self[2]]

    @staticmethod
    def getIntegerArray(self) -> array.array:
#        if (len(self.data) % 8) != 0:
        if (len(self[2]) % 8) != 0:
            raise TypeError("not an integer array")
        arr = array.array("l")
#        arr.frombytes(self.data)
        arr.frombytes(self[2])
        return arr

    @staticmethod
    def getFloatArray(self) -> array.array:
#        if (len(self.data) % 4) != 0:
        if (len(self[2]) % 4) != 0:
            raise TypeError("not a float array")
        arr = array.array("f")
#        arr.frombytes(self.data)
        arr.frombytes(self[2])
        return arr

    @staticmethod
    def getDoubleArray(self) -> array.array:
#        if (len(self.data) % 8) != 0:
        if (len(self[2]) % 8) != 0:
            raise TypeError("not a double array")
        arr = array.array("d")
#        arr.frombytes(self.data)
        arr.frombytes(self[2])
        return arr

    @staticmethod
    def getStringArray(self) -> List[str]:
#        size = int.from_bytes(self.data[0:4], byteorder="little", signed=False)
        size = int.from_bytes(self[2][0:4], byteorder="little", signed=False)
#        if size > ((len(self.data) - 4) / 4):
        if size > ((len(self[2]) - 4) / 4):
            raise TypeError("not a string array")
        arr = []
        pos = 4
        for i in range(size):
#            val, pos = self._readInnerString(pos)
            val, pos = DataLogRecord._readInnerString(self, pos)
            arr.append(val)
        return arr

    @staticmethod
    def getMsgPack(self):
#        return msgpack.unpackb(self.data)
        return msgpack.unpackb(self[2])

    @staticmethod
    def _readInnerString(self, pos: int) -> str:
        size = int.from_bytes(
#            self.data[pos : pos + 4], byteorder="little", signed=False
            self[2][pos : pos + 4], byteorder="little", signed=False
        )
        end = pos + 4 + size
#        if end > len(self.data):
        if end > len(self[2]):
            raise TypeError("invalid string size")
#        return str(self.data[pos + 4 : end], encoding="utf-8"), end
        return str(self[2][pos + 4 : end], encoding="utf-8"), end


class DataLogIterator:
    """DataLogReader iterator."""

    def __init__(self, buf: SupportsBytes, pos: int):
        self.buf = buf
        self.pos = pos

    def __iter__(self):
        return self

    def _readVarInt(self, pos: int, len: int) -> int:

# maybe use struct unpack here?
        if len == 1:
#            raise Exception("blarg1") 
            return  self.buf[pos]
        elif len == 2:
#           raise Exception("blarg2") 
            return  self.buf[pos] | self.buf[pos + 1] << 8
            # return int.frombytes()
        elif len == 3:
            # does this ever actually happen?
            return  self.buf[pos] | self.buf[pos + 1] << 8 | self.buf[pos + 2] << 16
#            raise Exception("blarg3") 
        else:            # len == 4
#            raise Exception("blarg4") 
            return  self.buf[pos] | self.buf[pos + 1] << 8 | self.buf[pos + 2] << 16 | self.buf[pos + 3] << 24

        # val = 0
        # for i in range(len):
        #     val |= self.buf[pos + i] << (i * 8)
        # return val

# try optimizing this function so it does less arithmetic
# try returning a tuple instead

    def __next__(self) -> Tuple[Any, Any, Any]:
        if len(self.buf) < (self.pos + 4):
            raise StopIteration
        # for our logs entryLen is 1 or 2 because we have >255 columns
        entryLen = (self.buf[self.pos] & 0x3) + 1
        # sizeLen is only ever 1, 2, 3 or 4.
        # actually sizeLen in practice is always 2
        sizeLen = ((self.buf[self.pos] >> 2) & 0x3) + 1
        timestampLen = ((self.buf[self.pos] >> 4) & 0x7) + 1
        headerLen = 1 + entryLen + sizeLen + timestampLen
        if len(self.buf) < (self.pos + headerLen):
            raise StopIteration
        # for now don't actually parse anything
        # to see how long this takes
        # taking two of the readVarInt calls out makes this 19s instead of 38, 50% !
        entry = self._readVarInt(self.pos + 1, entryLen)
#        entry = 0
        size = self._readVarInt(self.pos + 1 + entryLen, sizeLen)
        timestamp = self._readVarInt(self.pos + 1 + entryLen + sizeLen, timestampLen)
#        timestamp = 0
        if len(self.buf) < (self.pos + headerLen + size):
            raise StopIteration
#        record = DataLogRecord(
        record = (
            entry,
            timestamp,
            self.buf[self.pos + headerLen : self.pos + headerLen + size],
        )
        self.pos += headerLen + size
        return record


class DataLogReader:
    """Data log reader (reads logs written by the DataLog class)."""

    def __init__(self, buf: SupportsBytes):
        self.buf = buf

    def __bool__(self):
        return self.isValid()

    def isValid(self) -> bool:
        """Returns true if the data log is valid (e.g. has a valid header)."""
        return (
            len(self.buf) >= 12
            and self.buf[0:6] == b"WPILOG"
            and self.getVersion() >= 0x0100
        )

    def getVersion(self) -> int:
        """Gets the data log version. Returns 0 if data log is invalid.

        @return Version number; most significant byte is major, least significant is
            minor (so version 1.0 will be 0x0100)"""
        if len(self.buf) < 12:
            return 0
        return int.from_bytes(self.buf[6:8], byteorder="little", signed=False)

    def getExtraHeader(self) -> str:
        """Gets the extra header data.

        @return Extra header data
        """
        if len(self.buf) < 12:
            return ""
        size = int.from_bytes(self.buf[8:12], byteorder="little", signed=False)
        return str(self.buf[12 : 12 + size], encoding="utf-8")

    def __iter__(self) -> DataLogIterator:
        extraHeaderSize = int.from_bytes(
            self.buf[8:12], byteorder="little", signed=False
        )
        return DataLogIterator(self.buf, 12 + extraHeaderSize)


if __name__ == "__main__":
    from datetime import datetime
    import mmap
    import sys

    if len(sys.argv) != 2:
        print("Usage: datalog.py <file>", file=sys.stderr)
        sys.exit(1)

    with open(sys.argv[1], "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = DataLogReader(mm)
        if not reader:
            print("not a log file", file=sys.stderr)
            sys.exit(1)

        entries = {}
        for record in reader:
            timestamp = record.timestamp / 1000000
            if record.isStart():
                try:
                    data = record.getStartData()
                    print(
                        f"Start({data.entry}, name='{data.name}', type='{data.type}', metadata='{data.metadata}') [{timestamp}]"
                    )
                    if data.entry in entries:
                        print("...DUPLICATE entry ID, overriding")
                    entries[data.entry] = data
                except TypeError as e:
                    print("Start(INVALID)")
            elif record.isFinish():
                try:
                    entry = record.getFinishEntry()
                    print(f"Finish({entry}) [{timestamp}]")
                    if entry not in entries:
                        print("...ID not found")
                    else:
                        del entries[entry]
                except TypeError as e:
                    print("Finish(INVALID)")
            elif record.isSetMetadata():
                try:
                    data = record.getSetMetadataData()
                    print(f"SetMetadata({data.entry}, '{data.metadata}') [{timestamp}]")
                    if data.entry not in entries:
                        print("...ID not found")
                except TypeError as e:
                    print("SetMetadata(INVALID)")
            elif record.isControl():
                print("Unrecognized control record")
            else:
                print(f"Data({record.entry}, size={len(record.data)}) ", end="")
                entry = entries.get(record.entry, None)
                if entry is None:
                    print("<ID not found>")
                    continue
                print(f"<name='{entry.name}', type='{entry.type}'> [{timestamp}]")

                try:
                    # handle systemTime specially
                    if entry.name == "systemTime" and entry.type == "int64":
                        dt = datetime.fromtimestamp(record.getInteger() / 1000000)
                        print("  {:%Y-%m-%d %H:%M:%S.%f}".format(dt))
                        continue

                    if entry.type == "double":
                        print(f"  {record.getDouble()}")
                    elif entry.type == "int64":
                        print(f"  {record.getInteger()}")
                    elif entry.type == "string" or entry.type == "json":
                        print(f"  '{record.getString()}'")
                    elif entry.type == "boolean":
                        print(f"  {record.getBoolean()}")
                    elif entry.type == "boolean[]":
                        arr = record.getBooleanArray()
                        print(f"  {arr}")
                    elif entry.type == "double[]":
                        arr = record.getDoubleArray()
                        print(f"  {arr}")
                    elif entry.type == "float[]":
                        arr = record.getFloatArray()
                        print(f"  {arr}")
                    elif entry.type == "int64[]":
                        arr = record.getIntegerArray()
                        print(f"  {arr}")
                    elif entry.type == "string[]":
                        arr = record.getStringArray()
                        print(f"  {arr}")
                except TypeError as e:
                    print("  invalid")
