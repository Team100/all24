import struct
import timeit

# frombytes is the fastest except for one byte

REPS = 10000000

input1 = b'\x64'
input2 = b'\x64\xd8'
#input3 = b'\x64\xd8\x64'
input4 = b'\x64\xd8\x64\x3f'

def unpack1():
#    return struct.unpack('<B', input1)
    return struct.unpack('<B', input4[0:1])

def read1():
#    return input1[0]
    return input4[0]

def frombytes1():
#    return int.from_bytes(input1, 'little')
    return int.from_bytes(input4[0:1], 'little')

def unpack2():
#    return struct.unpack('<H', input2)
    return struct.unpack('<H', input4[0:2])

def read2():
#    return  input2[0] | input2[1] << 8
    return  input4[0] | input4[1] << 8

def frombytes2():
#    return int.from_bytes(input2, 'little')
    return int.from_bytes(input4[0:2], 'little')

# x3 = struct.unpack('<b', input3)

def unpack4():
#    return struct.unpack('<I', input4)
    return struct.unpack('<I', input4[0:4])

def read4():
    return  input4[0] | input4[1] << 8 | input4[2] << 16 | input4[3] << 24

def frombytes4():
#    return int.from_bytes(input4, 'little')
    return int.from_bytes(input4[0:4], 'little')

# 1.1
timer = timeit.Timer(unpack1)
print("unpack 1    ", timer.timeit(number=REPS))

# 0.62
timer = timeit.Timer(read1)
print("read 1      ", timer.timeit(number=REPS))

timer = timeit.Timer(frombytes1)
print("frombytes 1 ", timer.timeit(number=REPS))

# 1.22
timer = timeit.Timer(unpack2)
print("unpack 2    ", timer.timeit(number=REPS))

# 1.28
timer = timeit.Timer(read2)
print("read 2      ", timer.timeit(number=REPS))

timer = timeit.Timer(frombytes2)
print("frombytes 2 ", timer.timeit(number=REPS))

# 1.24
timer = timeit.Timer(unpack4)
print("unpack 4    ", timer.timeit(number=REPS))

# 2.73
timer = timeit.Timer(read4)
print("read 4      ", timer.timeit(number=REPS))

timer = timeit.Timer(frombytes4)
print("frombytes 4 ", timer.timeit(number=REPS))
