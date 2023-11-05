import log2df
import scan
import timeit
import datalog2
import mmap

# a very large file, 250MB
FILE = "FRC_20230815_164534.wpilog"

# log2df.log2df(FILE)

REPS = 1

def scan3():
    # just scan the file without doing anything else
    # takes 42 sec (!)
    # removing the object instantiation of each record makes it take 39 sec, so
    # a little under 10% savings.
    with open(FILE, "r") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        reader = datalog2.DataLogReader(mm)
        if not reader.isValid():
            return {}
        # i = 0
        for record in reader:
            pass
            # if record.isStart():
            #     print("start ", i, " ", record.getStartData().name)
            # i += 1


def test1():
    # returns a dataframe
    # takes 387 sec (!)
    log2df.log2df(FILE)

def test2():
    # returns a dict, takes 60 sec (!)
    # TODO change the dict it returns
    x = scan.scan(FILE)

def test4():
    x = scan.getKeyCounts(FILE, 1000)
    for item in x.items():
        print(item[1], item[0])

# timer = timeit.Timer(test1)
# print(timer.timeit(number=REPS))

timer = timeit.Timer(test2)
print(timer.timeit(number=REPS))

# timer = timeit.Timer(scan3)
# print(timer.timeit(number=REPS))

# timer = timeit.Timer(test4)
# print(timer.timeit(number=REPS))
