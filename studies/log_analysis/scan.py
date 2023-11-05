import datalog
import mmap
from typing import Any, Dict, List

# stop looking at depth
def getKeyCounts(filename: str, depth: int) -> Dict[str, int]:
    with open(filename, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = datalog.DataLogReader(mm)
        if not reader.isValid():
            return {}
        entries = {}  # to look up names, key = 'entry', value = start data
        i = 0;
        result = {}
        for record in reader:
            # read the whole file
            # i += 1
            # if i > depth:
            #     return result 
            if record.isStart():
                data = record.getStartData()
                result[data.name] = 0
                entries[data.entry] = data
            elif record.isFinish():  # never happens
                continue
            elif record.isSetMetadata():  # never happens
                continue
            elif record.isControl():  # never happens
                continue
            else: # normal data record
                entry = entries.get(record.entry, None)
                if entry is None:
                    continue
                result[entry.name] += 1

        return result


def scan(filename: str, columns: List[str] = []) -> dict[str, tuple[List[float], List[float]]]:
    with open(filename, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = datalog.DataLogReader(mm)
        if not reader.isValid():
            return {}
        # to look up names, key = 'entry', value = start data
        entries = {} 
        # key series name, value = two arrays
        series = {}
        for record in reader:
            if record.isStart():
                data = record.getStartData()
                if len(columns) == 0 or data.name in columns:
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
                if entry.name not in series:
                    series[entry.name] = ([],[]) # x,y
                # every series has a timestamp
                series[entry.name][0].append(record.timestamp)
                if entry.type == "double":
                    series[entry.name][1].append(record.getDouble())
                elif entry.type == "int64":
                    series[entry.name][1].append(record.getInteger())
                elif entry.type == "string":
                    series[entry.name][1].append(record.getString())
                elif entry.type == "json":
                    series[entry.name][1].append(record.getString())
                elif entry.type == "boolean":
                    series[entry.name][1].append(record.getBoolean())
                elif entry.type == "msgpack":
                    series[entry.name][1].append(record.getMsgPack())
    # TODO: do something with arrays?
    # elif entry.type == 'boolean[]':
    # elif entry.type == 'double[]':
    # elif entry.type == 'float[]':
    # elif entry.type == 'int64[]':
    # elif entry.type == 'string[]':

    return series
