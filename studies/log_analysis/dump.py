import log2df
import scan
import matplotlib.pyplot as plt

FILE = "FRC_20230205_004848.wpilog"

df = log2df.log2df(FILE)

# print(df)
print(df.columns)

plt.plot(df.index, '+')
plt.show()

arrays = scan.scan(FILE)

print (arrays.keys())

plt.plot(arrays['systemTime'][0], arrays['systemTime'][1])
plt.show()