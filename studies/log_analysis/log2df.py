import datalog
import mmap
import pandas as pd


def log2df(filename: str) -> pd.DataFrame:
    with open(filename, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = datalog.DataLogReader(mm)
        if not reader.isValid():
            return pd.DataFrame
        entries = {}  # to look up names, key = 'entry', value = start data
        rows = {}  # key = index = timestamp, value = dictionary of columns
        for record in reader:
            if record.isStart():
                data = record.getStartData()
                entries[data.entry] = data
            elif record.isFinish():  # never happens
                continue
            elif record.isSetMetadata():  # never happens
                continue
            elif record.isControl():  # never happens
                continue
            else:  # normal data record
                entry = entries.get(record.entry, None)
                if entry is None:
                    continue
                if record.timestamp not in rows:
                    rows[record.timestamp] = {"timestamp": record.timestamp}
                if entry.type == "double":
                    rows[record.timestamp][entry.name] = record.getDouble()
                elif entry.type == "int64":
                    rows[record.timestamp][entry.name] = record.getInteger()
                elif entry.type == "string":
                    rows[record.timestamp][entry.name] = record.getString()
                elif entry.type == "json":
                    rows[record.timestamp][entry.name] = record.getString()
                elif entry.type == "boolean":
                    rows[record.timestamp][entry.name] = record.getBoolean()
                elif entry.type == "msgpack":
                    rows[record.timestamp][entry.name] = record.getMsgPack()
    # TODO: do something with arrays?
    # elif entry.type == 'boolean[]':
    # elif entry.type == 'double[]':
    # elif entry.type == 'float[]':
    # elif entry.type == 'int64[]':
    # elif entry.type == 'string[]':

    df = pd.DataFrame.from_records(list(rows.values()))
    df.set_index("timestamp", inplace=True)
    df.pad(inplace=True)  # "last observation carried forward" for all timestamps
    return df
