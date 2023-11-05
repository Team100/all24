import cProfile
import io
import pstats
from pstats import SortKey


import log2df

with cProfile.Profile() as pr:
    for i in range(10):
        df = log2df.log2df("FRC_20230205_004848.wpilog")
    s = io.StringIO()
    sortby = SortKey.CUMULATIVE
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    ps.print_callers()
    print(s.getvalue())
