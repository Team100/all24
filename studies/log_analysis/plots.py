import log2df
import scan
import timeit
import datalog2
import mmap
import matplotlib.pyplot as plt

FILE = "FRC_20230930_195659.wpilog"

keyCounts = scan.getKeyCounts(FILE, 1000)
cols = []
for item in keyCounts.items():
#    print(item[1], item[0])
    if item[1] > 100:
        cols.append(item[0])

cols.sort()

for col in cols:
    print(col)


#x = scan.scan(FILE,  'NT:/SmartDashboard/Robot Container/Heading Degrees')
x = scan.scan(FILE)

for figidx in range(len(cols)//5+1):
    fig, axs = plt.subplots(5, sharex=True)
    #fig.set_figheight(20)
    for idx, col in enumerate(cols[figidx*5:(figidx+1)*5]):
        series = x[col]

        # we want about 500 points so
        step = len(series[0])//500+1

        timestamp = [t/1000000 for t in series[0][::step]]
        heading = series[1][::step]
        axs[idx].plot(timestamp, heading)
        axs[idx].set_xlabel('timestamp (sec)')
        axs[idx].title.set_text(col)
        plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)

plt.show()