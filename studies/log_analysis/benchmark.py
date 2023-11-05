"""compare tuple return, object return, and outvar"""
import timeit
from collections import namedtuple

REPS = 1000000


class ReturnVal:
    def __init__(self, item1: int, item2: int):
        self.item1 = item1
        self.item2 = item2


def test1():
    return ReturnVal(1, 2)


def test2():
    return (1, 2)


def test3(val: ReturnVal):
    val.item1 = 1
    val.item2 = 2


ReturnTuple = namedtuple("ReturnTuple", ['a','b'])

def test4():
    return ReturnTuple(1,2)

# 0.246
print("return an object")
timer = timeit.Timer(test1)
print(timer.timeit(number=REPS))

# 0.045
print("return a tuple")
timer = timeit.Timer(test2)
print(timer.timeit(number=REPS))

# 0.144
print("outvar")
value = ReturnVal(0, 0)
timer = timeit.Timer(lambda: test3(value))
print(timer.timeit(number=REPS))

# 0.237
print("named tuple")
timer = timeit.Timer(test4)
print(timer.timeit(number=REPS))
